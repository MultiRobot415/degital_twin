# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""位置制御ノード：mocap pose を見て、目標位置へ飛ぶ /cmd_vel を出す。

これが「デジタルツインの真ん中」。実機でもSimでも同じこのノードを使う:

    /tello/pose (PoseStamped)        ← 今どこにいるか（Simの真値 or 実機mocap）
        │
        ▼  目標位置との誤差を P制御し、機体座標に直す
    /cmd_vel  (Twist, 正規化 -1〜1)   → ドローン（Sim or 実機）へ

teleop_twist_keyboard を置き換える「本番のロジック」。

目標位置はパラメータで与える（起動時に変更可）:
    ros2 run drone_control position_controller --ros-args \
        -p goal_x:=1.0 -p goal_y:=-1.0 -p goal_z:=1.3

RViz の "2D Goal Pose" で /goal_pose を流すと、XY目標をその場で変えられる（高さは維持）。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """クォータニオン → ヨー角[rad]（Zまわりの回転）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller")

        # --- パラメータ（目標位置・ゲイン）。起動時に -p で変更できる ---
        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("goal_z", 1.2)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("kp_xy", 0.8)   # 水平の比例ゲイン
        self.declare_parameter("kp_z", 1.0)    # 高さの比例ゲイン
        self.declare_parameter("kp_yaw", 1.0)  # ヨーの比例ゲイン
        # 安全: 起動直後に勝手に飛ばさない。True なら「最初のposeを受けた地点を
        # ゴールにして“その場ホールド”」で始まり、/goal_pose(RVizの2D Goal Pose)が
        # 来て初めて動く。固定ゴールへ即追従したいときだけ false にして goal_* を使う。
        self.declare_parameter("hold_on_start", True)
        # 離陸ゲート高度[m]。床(z<この値)にいる間はコントローラは指令を出さず、
        # SDKのtakeoffに任せる。連続rc（特にz上昇）を離陸前に送るとTelloがtakeoffを
        # 弾いて離陸しなくなるため。持ち上がってこの高度を超えたら制御を引き継ぐ。
        self.declare_parameter("takeoff_height", 0.3)

        self.hold_on_start = bool(self.get_parameter("hold_on_start").value)
        self.goal = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
            self.get_parameter("goal_z").value,
        ], dtype=float)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

        # --- 状態 ---
        self.have_pose = False
        self.goal_ready = not self.hold_on_start  # hold時は最初のpose受信で確定する
        self.pos = np.zeros(3)
        self.yaw = 0.0

        # --- I/O：pose を購読し、cmd_vel を出す ---
        self.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)  # RViz から（任意）
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- ゴール可視化：今どこを目標に飛んでいるかを RViz に出す ---
        # 全機まとめて 1つの MarkerArray display（/goal_markers）で見えるよう、
        # 絶対トピックに機体ごと固有 ns で publish。late-join RViz 用に latched。
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self._marker_ns = self.get_namespace().strip("/") or "goal"
        goal_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.goal_marker_pub = self.create_publisher(MarkerArray, "/goal_markers", goal_qos)
        self.create_timer(0.2, self._publish_goal_marker)  # 5Hz で現在ゴールを送る

        # --- 制御ループ：50Hz で回す（mocap更新とは非同期でOK）---
        self.create_timer(0.02, self._control_step)
        # --- デバッグ：今どこにいる/どこへ行く/誤差/出力 を 2Hz で1行に出す ---
        # 「目標値は合っているか」「poseは正しいか」「向きは合っているか」を突き合わせで確認する用。
        self._last_cmd = None  # 直近に publish した cmd_vel（ログ用）
        self.create_timer(0.5, self._debug_log)
        mode = "現在地ホールド開始（/goal_pose 待ち）" if self.hold_on_start \
            else f"固定ゴール追従 goal={self.goal.tolist()}"
        self.get_logger().info(f"position controller up. {mode}")

    def _on_pose(self, msg: PoseStamped):
        """今の位置・ヨーを更新。hold_on_start なら最初の受信地点をゴールに固定。"""
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

        if not self.goal_ready:
            # 起動後はじめてのpose → その場(xy)を目標にしてホールド（勝手に飛ばない）。
            # ただし高さは“床の高さ”ではなく設定飛行高度(goal_zパラメータ)を保つ。
            # ここで pos.copy() をそのまま使うと、離陸前(床≈z0.0)の高さがゴールになり、
            # 離陸しても即降下→着地してしまう（xyも地面効果で追えない）。
            self.goal = self.pos.copy()
            self.goal[2] = float(self.get_parameter("goal_z").value)
            self.goal_yaw = self.yaw
            self.goal_ready = True
            self.get_logger().info(
                f"現在地でホールド開始 goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}, "
                f"{self.goal[2]:.2f}) yaw={self.goal_yaw:.2f} （高さは飛行高度goal_zを保持）"
            )

    def _on_goal(self, msg: PoseStamped):
        """RViz の 2D Goal Pose 等で XY 目標を更新（高さは現状維持）。"""
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y
        self.goal_ready = True  # hold中でも明示ゴールが来たら動いてよい
        self.get_logger().info(f"new goal xy=({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def _control_step(self):
        """誤差 → 機体座標 → P制御 → /cmd_vel。"""
        if not self.have_pose or not self.goal_ready:
            return  # まだ位置が来てない / ゴール未確定（ホールド待ち）

        # 離陸ゲート：床にいる間は一切指令を出さない（rcでtakeoffを妨げないため）。
        # SDKのtakeoffが機体を持ち上げ、takeoff_heightを超えたら以降の制御を引き継ぐ。
        takeoff_h = self.get_parameter("takeoff_height").value
        if self.pos[2] < takeoff_h:
            self._last_cmd = None
            return

        kp_xy = self.get_parameter("kp_xy").value
        kp_z = self.get_parameter("kp_z").value
        kp_yaw = self.get_parameter("kp_yaw").value
        goal_yaw = self.goal_yaw

        # 1) 世界座標での位置誤差
        err_w = self.goal - self.pos

        # 2) 機体座標へ回す（ヨーの逆回転）。
        #    Sim 側は body速度を yaw で世界へ回しているので、その逆変換にあたる。
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ex_body = c * err_w[0] + s * err_w[1]   # 前後方向の誤差
        ey_body = -s * err_w[0] + c * err_w[1]  # 左右方向の誤差
        ez = err_w[2]                            # 上下はそのまま

        # 3) P制御 → 正規化 cmd_vel（[-1,1] に飽和）
        twist = Twist()
        twist.linear.x = float(np.clip(kp_xy * ex_body, -1.0, 1.0))
        twist.linear.y = float(np.clip(kp_xy * ey_body, -1.0, 1.0))
        twist.linear.z = float(np.clip(kp_z * ez, -1.0, 1.0))

        # 4) ヨーも目標へ（最短角で）
        yaw_err = math.atan2(math.sin(goal_yaw - self.yaw), math.cos(goal_yaw - self.yaw))
        twist.angular.z = float(np.clip(kp_yaw * yaw_err, -1.0, 1.0))

        self.cmd_pub.publish(twist)
        self._last_cmd = twist  # デバッグログ用に保持

    def _debug_log(self):
        """今の pos / goal / 世界誤差 / 機体誤差 / 出力cmd を1行で出す（2Hz）。

        突き合わせなしで内部状態を確認するための可視化。挙動は変えない。
          pos      : mocap から受け取った現在位置（＝制御が使う self.pos）
          goal     : 今向かっている目標（self.goal）
          err_w    : 世界座標の位置誤差 goal - pos（xy）
          err_body : yaw で機体座標に回した誤差（linear.x/y の元）
          cmd      : 直近に publish した正規化 cmd_vel
        """
        if not self.have_pose or not self.goal_ready:
            # まだ pose 未受信 / ゴール未確定（ホールド待ち）→ 理由を出す
            self.get_logger().info(
                f"[待機] have_pose={self.have_pose} goal_ready={self.goal_ready} "
                f"(pose未受信ならmocap/ID、goal未確定ならhold_on_start中で目標未送信)"
            )
            return

        takeoff_h = self.get_parameter("takeoff_height").value
        if self.have_pose and self.pos[2] < takeoff_h:
            self.get_logger().info(
                f"[離陸ゲート] pos_z={self.pos[2]:+.2f} < takeoff_height={takeoff_h:.2f} "
                f"→ 指令停止中。Tのtakeoffで持ち上がれば制御開始。goal_z={self.goal[2]:+.2f}"
            )
            return

        err_w = self.goal - self.pos
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ex_body = c * err_w[0] + s * err_w[1]
        ey_body = -s * err_w[0] + c * err_w[1]
        cmd = self._last_cmd
        cmd_str = (f"x{cmd.linear.x:+.2f},y{cmd.linear.y:+.2f},z{cmd.linear.z:+.2f}"
                   if cmd is not None else "none")
        dist = float(np.hypot(err_w[0], err_w[1]))
        self.get_logger().info(
            f"pos=({self.pos[0]:+.2f},{self.pos[1]:+.2f},{self.pos[2]:+.2f}) "
            f"yaw={self.yaw:+.2f} "
            f"goal=({self.goal[0]:+.2f},{self.goal[1]:+.2f},{self.goal[2]:+.2f}) "
            f"err_w=({err_w[0]:+.2f},{err_w[1]:+.2f}) dist={dist:.2f} "
            f"err_body=({ex_body:+.2f},{ey_body:+.2f}) cmd=({cmd_str})"
        )

    def _publish_goal_marker(self):
        """現在の目標 self.goal を RViz に表示（球＝目標点、細い線＝現在地→目標）。"""
        if not self.goal_ready:
            return  # まだゴール未確定（hold で最初の pose 待ち）
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = self.world_frame
        sphere.header.stamp = stamp
        sphere.ns = f"{self._marker_ns}_goal"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(self.goal[0])
        sphere.pose.position.y = float(self.goal[1])
        sphere.pose.position.z = float(self.goal[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 1.0, 0.2, 0.9
        arr.markers.append(sphere)

        # 目標座標のテキスト（数値そのものを RViz 上に出す）
        text = Marker()
        text.header.frame_id = self.world_frame
        text.header.stamp = stamp
        text.ns = f"{self._marker_ns}_goal_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(self.goal[0])
        text.pose.position.y = float(self.goal[1])
        text.pose.position.z = float(self.goal[2]) + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.15
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"{self._marker_ns} goal ({self.goal[0]:+.2f}, {self.goal[1]:+.2f})"
        arr.markers.append(text)

        self.goal_marker_pub.publish(arr)


def main():
    rclpy.init()
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
