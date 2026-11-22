# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""ドローンショー（ルールベース）を 1 機ぶん実行するノード。Sim / 実機 共通。

    /<self>/pose          ─┐
    /formation/show       ─┤  ショーの中心 c・速度 ċ・隊形の回転角 φ・回転速度 φ̇（formation_commander が出す）
    /formation/enable     ─┘
           ▼ show_law.slot_target → consensus_law.compute_command（u = ṗ* − k(p − p*)）
    /<self>/cmd_vel, /<self>/formation_target（= この機体の目標 p_i*）

機体同士の情報は使わない（各機が同じルールで自分の目標を計算するだけ）。
enable の扱いは formation_agent / consensus_agent と同じ（false の間は cmd_vel を出さず、true→false で 0 指令）。

実行（通常は formation.launch.py formation:=drone_show から起動する）:
    ros2 run drone_control show_agent --ros-args -p name:=tello1 -p formation:=drone_show
"""

import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from drone_control import swarm_config
from drone_control.consensus_law import compute_command, leader_node
from drone_control.formation_agent import ENABLE_TOPIC, LATCHED_QOS
from drone_control.frames import quat_from_yaw, yaw_from_quat
from drone_control.show_law import slot_target

SHOW_TOPIC = "/formation/show"


class ShowAgent(Node):
    def __init__(self):
        super().__init__("show_agent")
        self.declare_parameter("name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.declare_parameter("formation", "drone_show")

        self.name = self.get_parameter("name").value
        self.swarm = swarm_config.load_swarm(self.get_parameter("swarm_file").value)
        path = swarm_config.formation_file(self.get_parameter("formation").value)
        self.spec = swarm_config.load_drone_show(path, self.swarm)
        if self.name not in self.spec.agents:
            raise ValueError(f"name='{self.name}' は設定に無い。候補: {self.swarm.names}")
        self.slot = self.spec.agents[self.name]
        self.world_frame = self.swarm.world_frame

        self.own: Optional[tuple] = None
        self.show: Optional[Odometry] = None
        self.enabled = False
        self._warned = set()

        self.create_subscription(PoseStamped, f"/{self.name}/pose", self._on_own, 10)
        self.create_subscription(Odometry, SHOW_TOPIC, self._on_show, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self._on_enable, LATCHED_QOS)
        self.cmd_pub = self.create_publisher(Twist, f"/{self.name}/cmd_vel", 10)
        self.target_pub = self.create_publisher(PoseStamped, f"/{self.name}/formation_target", 10)
        self.create_timer(1.0 / self.spec.rate_hz, self._step)

        self.get_logger().info(
            f"[{self.name}] drone_show '{self.spec.description}' | 半径 ρ={self.spec.radius}m "
            f"θ={self.slot.azimuth * 57.29578:.0f}° dz={self.slot.dz} k={self.spec.k} | "
            f"最小機体間距離（配置どおりのとき）={self.spec.min_separation():.2f}m。enable=true まで cmd_vel は出さない")

    def _frame_ok(self, frame_id: str, source: str) -> bool:
        if frame_id == self.world_frame:
            return True
        if (source, frame_id) not in self._warned:
            self._warned.add((source, frame_id))
            self.get_logger().error(f"{source} の frame_id='{frame_id}' は '{self.world_frame}' ではないので無視")
        return False

    def _on_own(self, m: PoseStamped):
        if self._frame_ok(m.header.frame_id, f"/{self.name}/pose"):
            p, q = m.pose.position, m.pose.orientation
            self.own = ((p.x, p.y, p.z), yaw_from_quat(q.x, q.y, q.z, q.w))

    def _on_show(self, m: Odometry):
        if self._frame_ok(m.header.frame_id, SHOW_TOPIC):
            self.show = m

    def _on_enable(self, m: Bool):
        if self.enabled and not m.data:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"[{self.name}] disable → 0 指令を送信")
        elif not self.enabled and m.data:
            self.get_logger().info(f"[{self.name}] enable")
        self.enabled = bool(m.data)

    def _step(self):
        missing = ([] if self.own is not None else [f"/{self.name}/pose"]) + \
                  ([] if self.show is not None else [SHOW_TOPIC])
        if missing:
            if self.enabled:
                self.get_logger().warn(f"[{self.name}] 入力待ち: {missing}", throttle_duration_sec=2.0)
            return

        sp, sv = self.show.pose.pose, self.show.twist.twist
        center = (sp.position.x, sp.position.y, sp.position.z)
        center_vel = (sv.linear.x, sv.linear.y, sv.linear.z)
        phase = yaw_from_quat(sp.orientation.x, sp.orientation.y, sp.orientation.z, sp.orientation.w)
        p_star, v_star = slot_target(self.slot, self.spec.radius, center, center_vel, phase, sv.angular.z)

        tgt = PoseStamped()
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.header.frame_id = self.world_frame
        tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = p_star
        q = quat_from_yaw(self.spec.goal_yaw)
        tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w = q
        self.target_pub.publish(tgt)

        if not self.enabled:
            return
        p_i, yaw_i = self.own
        out = compute_command(p_i, yaw_i, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.spec,
                              [leader_node(p_star, v_star)],
                              self.swarm.velocity_scale_xy, self.swarm.velocity_scale_z)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = out.body_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ShowAgent()
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
