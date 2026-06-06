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
from geometry_msgs.msg import PoseStamped, Twist


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

        self.goal = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
            self.get_parameter("goal_z").value,
        ], dtype=float)

        # --- 状態 ---
        self.have_pose = False
        self.pos = np.zeros(3)
        self.yaw = 0.0

        # --- I/O：pose を購読し、cmd_vel を出す ---
        self.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)  # RViz から（任意）
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- 制御ループ：50Hz で回す（mocap更新とは非同期でOK）---
        self.create_timer(0.02, self._control_step)
        self.get_logger().info(f"position controller up. goal={self.goal.tolist()}")

    def _on_pose(self, msg: PoseStamped):
        """今の位置・ヨーを更新。"""
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

    def _on_goal(self, msg: PoseStamped):
        """RViz の 2D Goal Pose 等で XY 目標を更新（高さは現状維持）。"""
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y
        self.get_logger().info(f"new goal xy=({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def _control_step(self):
        """誤差 → 機体座標 → P制御 → /cmd_vel。"""
        if not self.have_pose:
            return  # まだ位置が来てない

        kp_xy = self.get_parameter("kp_xy").value
        kp_z = self.get_parameter("kp_z").value
        kp_yaw = self.get_parameter("kp_yaw").value
        goal_yaw = self.get_parameter("goal_yaw").value

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
