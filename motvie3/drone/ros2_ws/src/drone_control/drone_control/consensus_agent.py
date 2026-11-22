# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""論文 第3章の合意フォーメーション（1次系）を 1 機ぶん実行するノード。Sim / 実機 共通。

情報の流れ（a_ij = 1 の相手だけを購読する = ネットワーク構造そのもの）:

    /<self>/pose                     ─┐
    /<近傍j>/pose                     ─┤  r_j
    /<近傍j>/formation_velocity       ─┤  ṙ_j（近傍が出している速度指令, world, m/s）
    /formation/leader（leader: true） ─┤  r_L, ṙ_L
    /formation/leader（全機）          ─┘  進行方向 ψ, ψ̇（隊形の向き d_i(t) を計算するためだけに使う）
                ▼ consensus_law.compute_command（式 3.5）
    /<self>/cmd_vel（正規化・機体座標）, /<self>/formation_velocity（u_i を近傍へ）,
    /<self>/formation_target（この機体が推定する目標位置）

隊形の向き（ψ）は Leader の情報だが、位置・速度とは別に全機が受け取る（簡略化）。
論文では Leader が直接つながる機体に隊形情報 d_i を与え、ネットワークで伝搬させる。
ここでは隊形情報だけ直接配信し、位置・速度の情報はネットワーク構造どおりにやり取りする。

enable の扱いは formation_agent と同じ:
    false の間は cmd_vel を出さない。formation_velocity は 0 を出す（近傍のフィードフォワードを 0 にする）
    true→false で 0 指令を 1 回出す（Tello は最後の rc を保持するため）

実行（通常は formation.launch.py formation:=ch3_net2 から起動する）:
    ros2 run drone_control consensus_agent --ros-args -p name:=tello1 -p formation:=ch3_net2
"""

import signal
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from drone_control import swarm_config
from drone_control.consensus_law import NodeState, compute_command, leader_node, offset_pair
from drone_control.formation_agent import ENABLE_TOPIC, LATCHED_QOS
from drone_control.frames import quat_from_yaw, yaw_from_quat

LEADER_TOPIC = "/formation/leader"


class ConsensusAgent(Node):
    def __init__(self):
        super().__init__("consensus_agent")
        self.declare_parameter("name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.declare_parameter("formation", "ch3_net2")

        self.name = self.get_parameter("name").value
        self.swarm = swarm_config.load_swarm(self.get_parameter("swarm_file").value)
        path = swarm_config.formation_file(self.get_parameter("formation").value)
        self.spec = swarm_config.load_consensus(path, self.swarm)
        if self.name not in self.spec.agents:
            raise ValueError(f"name='{self.name}' は設定に無い。候補: {self.swarm.names}")
        self.agent = self.spec.agents[self.name]
        self.world_frame = self.swarm.world_frame

        # --- 最新値 ---
        self.own: Optional[tuple] = None                                   # (p, yaw)
        self.nbr_pos: Dict[str, Optional[tuple]] = {j: None for j in self.agent.neighbors}
        self.nbr_vel: Dict[str, Optional[tuple]] = {j: None for j in self.agent.neighbors}
        self.leader: Optional[Odometry] = None
        self.enabled = False
        self._warned = set()

        # --- I/O ---
        self.create_subscription(PoseStamped, f"/{self.name}/pose", self._on_own, 10)
        for j in self.agent.neighbors:
            self.create_subscription(PoseStamped, f"/{j}/pose", lambda m, j=j: self._on_nbr_pose(j, m), 10)
            self.create_subscription(TwistStamped, f"/{j}/formation_velocity",
                                     lambda m, j=j: self._on_nbr_vel(j, m), 10)
        self.need_leader = self.agent.leader or self.spec.rotate_with_leader
        if self.need_leader:
            self.create_subscription(Odometry, LEADER_TOPIC, self._on_leader, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self._on_enable, LATCHED_QOS)
        self.cmd_pub = self.create_publisher(Twist, f"/{self.name}/cmd_vel", 10)
        self.vel_pub = self.create_publisher(TwistStamped, f"/{self.name}/formation_velocity", 10)
        self.target_pub = self.create_publisher(PoseStamped, f"/{self.name}/formation_target", 10)
        self.create_timer(1.0 / self.spec.rate_hz, self._step)

        self.get_logger().info(
            f"[{self.name}] 論文3章 合意フォーメーション '{self.spec.description}' | "
            f"Leader 直結={self.agent.leader} 近傍={list(self.agent.neighbors)} | "
            f"|d|={self.agent.distance}m θ={self.agent.azimuth * 57.29578:.0f}° "
            f"k={self.spec.k} rotate={self.spec.rotate_with_leader}。enable=true まで cmd_vel は出さない")

    # ------------------------------------------------------------------ 入力
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

    def _on_nbr_pose(self, j: str, m: PoseStamped):
        if self._frame_ok(m.header.frame_id, f"/{j}/pose"):
            p = m.pose.position
            self.nbr_pos[j] = (p.x, p.y, p.z)

    def _on_nbr_vel(self, j: str, m: TwistStamped):
        if self._frame_ok(m.header.frame_id, f"/{j}/formation_velocity"):
            v = m.twist.linear
            self.nbr_vel[j] = (v.x, v.y, v.z)

    def _on_leader(self, m: Odometry):
        if self._frame_ok(m.header.frame_id, LEADER_TOPIC):
            self.leader = m

    def _on_enable(self, m: Bool):
        if self.enabled and not m.data:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"[{self.name}] disable → 0 指令を送信")
        elif not self.enabled and m.data:
            self.get_logger().info(f"[{self.name}] enable")
        self.enabled = bool(m.data)

    # ------------------------------------------------------------------ 制御
    def _publish_velocity(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
        self.vel_pub.publish(msg)

    def _step(self):
        missing = [] if self.own is not None else [f"/{self.name}/pose"]
        missing += [f"/{j}/pose" for j, v in self.nbr_pos.items() if v is None]
        missing += [f"/{j}/formation_velocity" for j, v in self.nbr_vel.items() if v is None]
        if self.need_leader and self.leader is None:
            missing.append(LEADER_TOPIC)
        if missing:
            self._publish_velocity((0.0, 0.0, 0.0))  # 起動直後でも近傍が待たないように 0 を流す
            if self.enabled:
                self.get_logger().warn(f"[{self.name}] 入力待ち: {missing}", throttle_duration_sec=2.0)
            return

        heading, heading_rate = 0.0, 0.0
        if self.leader is not None:
            q = self.leader.pose.pose.orientation
            heading = yaw_from_quat(q.x, q.y, q.z, q.w)
            heading_rate = self.leader.twist.twist.angular.z
        rotate = self.spec.rotate_with_leader
        d_i, dd_i = offset_pair(self.agent, heading, heading_rate, rotate)

        others = []
        for j in self.agent.neighbors:
            d_j, dd_j = offset_pair(self.spec.agents[j], heading, heading_rate, rotate)
            others.append(NodeState(self.nbr_pos[j], self.nbr_vel[j], d_j, dd_j))
        if self.agent.leader:
            lp, lv = self.leader.pose.pose.position, self.leader.twist.twist.linear
            others.append(leader_node((lp.x, lp.y, lp.z), (lv.x, lv.y, lv.z)))

        p_i, yaw_i = self.own
        out = compute_command(p_i, yaw_i, d_i, dd_i, self.spec, others,
                              self.swarm.velocity_scale_xy, self.swarm.velocity_scale_z)

        tgt = PoseStamped()
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.header.frame_id = self.world_frame
        tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = out.target
        qx, qy, qz, qw = quat_from_yaw(self.spec.goal_yaw)
        tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w = qx, qy, qz, qw
        self.target_pub.publish(tgt)

        if not self.enabled:
            self._publish_velocity((0.0, 0.0, 0.0))
            return
        self._publish_velocity(out.world_velocity)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = out.body_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ConsensusAgent()
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
