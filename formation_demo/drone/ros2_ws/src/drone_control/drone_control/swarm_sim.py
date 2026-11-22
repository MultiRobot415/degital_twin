# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""N 機の Tello を模擬する軽量シミュレータ（Isaac Sim 不要・Python のみ）。

実機インフラ（mocap + pose_bridge + tello_swarm_driver）と「同じトピック・サービス」を出すので、
formation_agent / formation_commander は Sim と実機で無改造のまま使える:

    購読   /<name>/cmd_vel        (Twist, 正規化 -1〜1, 機体座標 ROS 規約)
    提供   /<name>/tello_action   (tello_msgs/TelloAction: takeoff / land / emergency / command)
    配信   /<name>/tello_response (tello_msgs/TelloResponse: 離着陸の完了通知)
           /<name>/flight_data    (tello_msgs/FlightData: h, bat のみ模擬)
           /<name>/pose + TF world-><name>        … emulate_mocap:=false（既定）
           /rigid_bodies（Motive 座標で模擬）       … emulate_mocap:=true
               → 実機と同じ pose_bridge が /<name>/pose に変換する。
                 座標変換（frames.py ①→②）の経路まで含めて Sim で検証できる。

運動モデルは drone/sim/tello_sim.py（Isaac Sim, B1）と同じ:
    v_cmd = [lx·MAX_VXY, ly·MAX_VXY, lz·MAX_VZ]（機体座標）, wz = az·MAX_WZ
            MAX_VXY / MAX_VZ = swarm.yaml の vehicle.velocity_scale（既定 1.0 / 0.6 = tello_sim と同じ）
    v ← v + (v_cmd − v)·(1 − e^{−dt/τ})          … 1次遅れ（tello_sim の SMOOTH=0.1@60Hz ≈ τ 0.15s）
    p ← p + R(yaw)·v·dt,  yaw ← yaw + wz·dt,  z ≥ 0（地面）
    重力・機体同士の衝突・通信遅延・rc 不感帯は模擬しない。

離着陸（実機 Tello の振る舞いに合わせた状態遷移）:
    LANDED --takeoff--> TAKING_OFF --(takeoff_height 到達)--> FLYING --land--> LANDING --> LANDED
    離着陸中は cmd_vel を無視（実機ドライバも応答待ちの間は rc を送らない）。
    LANDED 中も cmd_vel は無視。emergency はその場で LANDED（モーター停止 = 落下を即時に模擬）。
    最後に受けた cmd_vel は次が来るまで保持する（実機 Tello も最後の rc を保持する）。
"""

import math
import signal
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from mocap4r2_msgs.msg import RigidBodies, RigidBody
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction

from drone_control import swarm_config
from drone_control.frames import quat_from_yaw, ros_to_motive_yup, wrap_angle

LANDED, TAKING_OFF, FLYING, LANDING = "LANDED", "TAKING_OFF", "FLYING", "LANDING"


@dataclass
class SimDrone:
    name: str
    rigid_body_id: int
    pos: list
    yaw: float
    state: str = LANDED
    cmd: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])  # 保持中の正規化指令
    vel: list = field(default_factory=lambda: [0.0, 0.0, 0.0])       # 機体座標の速度 [m/s]


class SwarmSim(Node):
    def __init__(self):
        super().__init__("swarm_sim")
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.declare_parameter("emulate_mocap", False)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_wz", 3.0)           # [rad/s]
        self.declare_parameter("velocity_tau", 0.15)    # 1次遅れの時定数 [s]
        self.declare_parameter("takeoff_height", 0.8)   # [m]
        self.declare_parameter("takeoff_speed", 0.4)    # [m/s]
        self.declare_parameter("land_speed", 0.4)       # [m/s]

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.swarm = swarm_config.load_swarm(gp("swarm_file"))
        self.emulate_mocap = bool(gp("emulate_mocap"))
        # 正規化 1.0 のときの速度は swarm.yaml の vehicle.velocity_scale（consensus_agent と同じ値を使う）
        self.max_vxy, self.max_vz = self.swarm.velocity_scale_xy, self.swarm.velocity_scale_z
        self.max_wz = float(gp("max_wz"))
        self.tau = float(gp("velocity_tau"))
        self.takeoff_height = float(gp("takeoff_height"))
        self.takeoff_speed, self.land_speed = float(gp("takeoff_speed")), float(gp("land_speed"))
        self.world_frame = self.swarm.world_frame

        self.drones = {}
        self.pose_pubs, self.resp_pubs, self.fd_pubs = {}, {}, {}
        for d in self.swarm.drones:
            self.drones[d.name] = SimDrone(d.name, d.rigid_body_id, list(d.sim_initial_position),
                                           d.sim_initial_yaw)
            self.create_subscription(Twist, f"/{d.name}/cmd_vel",
                                     lambda m, n=d.name: self._on_cmd(n, m), 10)
            self.create_service(TelloAction, f"/{d.name}/tello_action",
                                lambda req, res, n=d.name: self._on_action(n, req, res))
            self.resp_pubs[d.name] = self.create_publisher(TelloResponse, f"/{d.name}/tello_response", 10)
            self.fd_pubs[d.name] = self.create_publisher(FlightData, f"/{d.name}/flight_data", 10)
            if not self.emulate_mocap:
                self.pose_pubs[d.name] = self.create_publisher(PoseStamped, f"/{d.name}/pose", 10)

        if self.emulate_mocap:
            # 実機 mocap ドライバと同じく best_effort で出す（pose_bridge 側も best_effort 購読）
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
            self.rb_pub = self.create_publisher(RigidBodies, "/rigid_bodies", qos)
            self.frame_number = 0
        else:
            self.tf_broadcaster = TransformBroadcaster(self)

        self._last_t = None
        self.create_timer(1.0 / float(gp("rate_hz")), self._step)
        self.create_timer(0.1, self._publish_flight_data)  # 実機の state も 10Hz

        out = (f"/rigid_bodies（Motive {'Y-up' if self.swarm.convert_y_up else 'Z-up'} 座標で模擬）"
               if self.emulate_mocap else "/<name>/pose + TF")
        self.get_logger().info(
            f"swarm_sim: {self.swarm.names} | 出力={out} | 速度係数 xy={self.max_vxy} z={self.max_vz} m/s | "
            f"全機 LANDED で開始（takeoff 待ち）")

    # ------------------------------------------------------------------ 入力
    def _on_cmd(self, name: str, msg: Twist):
        d = self.drones[name]
        if d.state != FLYING:
            return  # 地上・離着陸中は無視（実機ドライバも応答待ち中は rc を送らない）
        d.cmd = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]

    def _on_action(self, name: str, req, res):
        d = self.drones[name]
        cmd = req.cmd.strip()
        if cmd == "emergency":
            d.state, d.pos[2] = LANDED, 0.0
            d.cmd, d.vel = [0.0] * 4, [0.0] * 3
            self._respond(name, TelloResponse.OK, "ok")
            res.rc = res.OK
            return res
        if d.state in (TAKING_OFF, LANDING):
            self.get_logger().warn(f"[{name}] Busy, dropping '{cmd}'")
            res.rc = res.ERROR_BUSY
            return res

        res.rc = res.OK
        if cmd == "command":
            self._respond(name, TelloResponse.OK, "ok")
        elif cmd == "takeoff" and d.state == LANDED:
            d.state, d.cmd, d.vel = TAKING_OFF, [0.0] * 4, [0.0] * 3
        elif cmd == "land" and d.state == FLYING:
            d.state, d.cmd, d.vel = LANDING, [0.0] * 4, [0.0] * 3
        else:
            # 飛行中の takeoff・地上での land・未対応コマンドは実機同様 "error"
            self.get_logger().warn(f"[{name}] '{cmd}' は状態 {d.state} では実行できない/未対応 → error")
            self._respond(name, TelloResponse.ERROR, "error")
        return res

    def _respond(self, name: str, rc: int, text: str):
        msg = TelloResponse()
        msg.rc, msg.str = rc, text
        self.resp_pubs[name].publish(msg)

    # ------------------------------------------------------------------ 物理
    def _step(self):
        now = self.get_clock().now()
        dt = 0.0 if self._last_t is None else (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        dt = max(0.0, min(dt, 0.05))  # 停止からの復帰などで1ステップが飛びすぎないように

        for d in self.drones.values():
            if d.state == TAKING_OFF:
                d.pos[2] = min(self.takeoff_height, d.pos[2] + self.takeoff_speed * dt)
                if d.pos[2] >= self.takeoff_height:
                    d.state = FLYING
                    self._respond(d.name, TelloResponse.OK, "ok")
            elif d.state == LANDING:
                d.pos[2] = max(0.0, d.pos[2] - self.land_speed * dt)
                if d.pos[2] <= 0.0:
                    d.state = LANDED
                    self._respond(d.name, TelloResponse.OK, "ok")
            elif d.state == FLYING:
                self._integrate(d, dt)

        self._publish_poses(now)

    def _integrate(self, d: SimDrone, dt: float):
        n = [max(-1.0, min(1.0, v)) for v in d.cmd]
        v_cmd = (n[0] * self.max_vxy, n[1] * self.max_vxy, n[2] * self.max_vz)
        alpha = 1.0 - math.exp(-dt / self.tau) if self.tau > 0.0 else 1.0
        d.vel = [v + (vc - v) * alpha for v, vc in zip(d.vel, v_cmd)]

        c, s = math.cos(d.yaw), math.sin(d.yaw)
        vx_w = c * d.vel[0] - s * d.vel[1]
        vy_w = s * d.vel[0] + c * d.vel[1]
        d.pos[0] += vx_w * dt
        d.pos[1] += vy_w * dt
        d.pos[2] += d.vel[2] * dt
        if d.pos[2] < 0.0:            # 地面
            d.pos[2] = 0.0
            d.vel[2] = max(0.0, d.vel[2])
        d.yaw = wrap_angle(d.yaw + n[3] * self.max_wz * dt)

    # ------------------------------------------------------------------ 出力
    def _publish_poses(self, now):
        stamp = now.to_msg()
        if self.emulate_mocap:
            msg = RigidBodies()
            msg.header.stamp = stamp
            msg.header.frame_id = self.world_frame
            self.frame_number += 1
            msg.frame_number = self.frame_number
            for d in self.drones.values():
                p, q = tuple(d.pos), quat_from_yaw(d.yaw)
                if self.swarm.convert_y_up:
                    p, q = ros_to_motive_yup(p, q)  # 研究室の Motive と同じ Y-up で出す
                rb = RigidBody()
                rb.rigid_body_name = str(d.rigid_body_id)  # 実機ドライバも Streaming ID の文字列を入れる
                rb.pose.position.x, rb.pose.position.y, rb.pose.position.z = p
                rb.pose.orientation.x, rb.pose.orientation.y = q[0], q[1]
                rb.pose.orientation.z, rb.pose.orientation.w = q[2], q[3]
                msg.rigidbodies.append(rb)
            self.rb_pub.publish(msg)
            return

        for d in self.drones.values():
            qx, qy, qz, qw = quat_from_yaw(d.yaw)
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.world_frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = d.pos
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            self.pose_pubs[d.name].publish(ps)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.world_frame
            t.child_frame_id = d.name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = d.pos
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(t)

    def _publish_flight_data(self):
        stamp = self.get_clock().now().to_msg()
        for d in self.drones.values():
            fd = FlightData()
            fd.header.stamp = stamp
            fd.raw = f"sim;state:{d.state};"
            fd.sdk = FlightData.SDK_2_0
            fd.h = int(round(d.pos[2] * 100.0))
            fd.tof = fd.h
            fd.bat = 100
            self.fd_pubs[d.name].publish(fd)


def main():
    rclpy.init()
    node = SwarmSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C は端末と launch の両方から届くので、後片付けに入ったら 2 回目以降の割り込みは無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
