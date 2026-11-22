# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""1機に1つ起動するフォーメーション制御ノード（分散制御の単位）。

このノードが購読するのは「自分」「近傍（formation yaml の neighbors）」「基準（pinned のときだけ）」
の情報だけ。つまり ROS のトピック接続がそのまま機体間の情報グラフになる:

    /<self>/pose            ─┐
    /<近傍j>/pose  (j∈N_i)  ─┼─▶ formation_law.compute_command ─▶ /<self>/cmd_vel
    /formation/reference    ─┘   (pinned のときだけ購読)          └▶ /<self>/formation_target
    /formation/enable       ── 指令を出すかどうか（formation_commander が出す）

Sim でも実機でもこのノードは同じ（/<name>/pose を出すのが swarm_sim か pose_bridge かの違いだけ）。

動作:
    - enable=false の間は cmd_vel を出さない（target は出すので RViz で行き先を確認できる）
    - enable が true→false になった瞬間に 0 指令を1回出す
      （Tello は最後の rc を保持し続けるので、止めたいなら明示的に 0 を送る必要がある）
    - 必要な入力（自分・近傍の pose、pinned なら基準）が1度も来ていなければ何も出さない
    - frame_id が world_frame と違う pose/基準は捨てる（座標系の取り違え防止）

実行（通常は formation.launch.py から起動する）:
    ros2 run drone_control formation_agent --ros-args -p name:=tello1 -p formation:=leader_follower
"""

import signal
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

from drone_control import swarm_config
from drone_control.formation_law import NeighborState, compute_command
from drone_control.frames import quat_from_yaw, yaw_from_quat

# 基準・enable は「最後の値」を後から起動したノードにも届ける（latched）
LATCHED_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
REFERENCE_TOPIC = "/formation/reference"
ENABLE_TOPIC = "/formation/enable"


class FormationAgent(Node):
    def __init__(self):
        super().__init__("formation_agent")
        self.declare_parameter("name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.declare_parameter("formation", "reference_tracking")

        self.name = self.get_parameter("name").value
        swarm_path = self.get_parameter("swarm_file").value
        formation_path = swarm_config.formation_file(self.get_parameter("formation").value)
        self.swarm = swarm_config.load_swarm(swarm_path)
        self.formation = swarm_config.load_formation(formation_path, self.swarm)
        if self.name not in self.formation.agents:
            raise ValueError(f"name='{self.name}' は設定に無い。候補: {self.swarm.names}")
        self.agent = self.formation.agents[self.name]
        self.control = self.formation.control
        self.world_frame = self.swarm.world_frame

        # --- 状態（最新値を保持）---
        self.own_pose: Optional[tuple] = None                 # (p, yaw)
        self.nbr_pos: Dict[str, Optional[tuple]] = {j: None for j, _ in self.agent.neighbors}
        self.reference: Optional[tuple] = None
        self.enabled = False
        self._warned_frames = set()

        # --- I/O ---
        self.create_subscription(PoseStamped, f"/{self.name}/pose", self._on_own_pose, 10)
        for j, _ in self.agent.neighbors:
            self.create_subscription(PoseStamped, f"/{j}/pose",
                                     lambda msg, j=j: self._on_nbr_pose(j, msg), 10)
        if self.agent.pinned:
            self.create_subscription(PoseStamped, REFERENCE_TOPIC, self._on_reference, LATCHED_QOS)
        self.create_subscription(Bool, ENABLE_TOPIC, self._on_enable, LATCHED_QOS)
        self.cmd_pub = self.create_publisher(Twist, f"/{self.name}/cmd_vel", 10)
        self.target_pub = self.create_publisher(PoseStamped, f"/{self.name}/formation_target", 10)

        self.create_timer(1.0 / self.control.rate_hz, self._step)

        nbrs = ", ".join(f"{j}(w={w})" for j, w in self.agent.neighbors) or "なし"
        self.get_logger().info(
            f"[{self.name}] formation='{self.formation.description}' ({formation_path}) | "
            f"pinned={self.agent.pinned} neighbors=[{nbrs}] offset(ROS)={self.agent.offset} | "
            f"rate={self.control.rate_hz}Hz。/formation/enable=true まで cmd_vel は出さない")

    # ------------------------------------------------------------------ 入力
    def _frame_ok(self, msg: PoseStamped, source: str) -> bool:
        if msg.header.frame_id == self.world_frame:
            return True
        key = (source, msg.header.frame_id)
        if key not in self._warned_frames:
            self._warned_frames.add(key)
            self.get_logger().error(
                f"{source} の frame_id='{msg.header.frame_id}' は '{self.world_frame}' ではないので無視する")
        return False

    def _on_own_pose(self, msg: PoseStamped):
        if not self._frame_ok(msg, f"/{self.name}/pose"):
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.own_pose = ((p.x, p.y, p.z), yaw_from_quat(q.x, q.y, q.z, q.w))

    def _on_nbr_pose(self, j: str, msg: PoseStamped):
        if not self._frame_ok(msg, f"/{j}/pose"):
            return
        p = msg.pose.position
        self.nbr_pos[j] = (p.x, p.y, p.z)

    def _on_reference(self, msg: PoseStamped):
        if not self._frame_ok(msg, REFERENCE_TOPIC):
            return
        p = msg.pose.position
        self.reference = (p.x, p.y, p.z)

    def _on_enable(self, msg: Bool):
        if self.enabled and not msg.data:
            self.cmd_pub.publish(Twist())  # 0 指令（ホバリング）を明示的に送る
            self.get_logger().info(f"[{self.name}] disable → 0 指令を送信")
        elif not self.enabled and msg.data:
            self.get_logger().info(f"[{self.name}] enable")
        self.enabled = bool(msg.data)

    # ------------------------------------------------------------------ 制御
    def _missing_inputs(self):
        missing = []
        if self.own_pose is None:
            missing.append(f"/{self.name}/pose")
        missing += [f"/{j}/pose" for j, v in self.nbr_pos.items() if v is None]
        if self.agent.pinned and self.reference is None:
            missing.append(REFERENCE_TOPIC)
        return missing

    def _step(self):
        missing = self._missing_inputs()
        if missing:
            if self.enabled:
                self.get_logger().warn(f"[{self.name}] 入力待ち: {missing}", throttle_duration_sec=2.0)
            return

        p_i, yaw_i = self.own_pose
        neighbors = [NeighborState(self.nbr_pos[j], self.formation.agents[j].offset, w)
                     for j, w in self.agent.neighbors]
        out = compute_command(p_i, yaw_i, self.agent, self.control, self.reference, neighbors)

        tgt = PoseStamped()
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.header.frame_id = self.world_frame
        tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = out.target
        qx, qy, qz, qw = quat_from_yaw(self.control.goal_yaw)
        tgt.pose.orientation.x, tgt.pose.orientation.y = qx, qy
        tgt.pose.orientation.z, tgt.pose.orientation.w = qz, qw
        self.target_pub.publish(tgt)

        if not self.enabled:
            return
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = out.body_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = FormationAgent()
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
