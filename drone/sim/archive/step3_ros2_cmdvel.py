# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フェーズ2 / ステップ3：ROS2 の /cmd_vel で Tello役の箱を飛ばす（①+傾き演出）。

step2 からの変更点は「入力源」だけ:
    step2: キーボード      → 速度指令
    step3: ROS2 /cmd_vel   → 速度指令   ← ここだけ差し替え
速度モデル（積分・回転・傾き演出）は step2 と完全に同じ。

実機 tello_ros と同じインターフェースに寄せている:
    /cmd_vel  (geometry_msgs/Twist, 正規化 -1〜1)
      linear.x  = 前後 (vx)
      linear.y  = 左右 (vy)
      linear.z  = 上下 (vz)
      angular.z = 旋回 (yaw rate)
    → 受け取った値を [-1,1] にクランプし、実機相当の最高速度にスケールする。
    → だから「実機を飛ばす teleop ノード」がそのまま Sim も飛ばせる。

実行方法（ROS2 を source してから venv の python で！）:

    source /home/utsubo/env_isaaclab/bin/activate
    source /opt/ros/humble/setup.bash
    python ~/research/drone/sim/step3_ros2_cmdvel.py

別端末から操作（標準のキーボード teleop で /cmd_vel を流す）:

    source /opt/ros/humble/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

オプション:
    --headless   GUI を出さない
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim を起動
# =============================================================================
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Step3: ROS2 /cmd_vel teleop")
parser.add_argument("--headless", action="store_true", help="GUIを出さない")
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

# =============================================================================
# 2) ここから core を import できる（rclpy は ROS2 を source 済みなら使える）
# =============================================================================
import os

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import open_stage

# --- アリーナ USD の場所 ----------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARENA_USD = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "env", "drone_arena.usd"))

# --- 動きのパラメータ（step2 と同じ意味）------------------------------------
START_POS = np.array([0.0, 0.0, 1.0])

MAX_VXY = 1.0     # 前後・左右の最大速度 [m/s]（正規化 1.0 のときの速度）
MAX_VZ = 0.6      # 上下の最大速度 [m/s]
MAX_WZ = 1.2      # ヨー最大角速度 [rad/s]
SMOOTH = 0.1      # 指令への追従の鈍さ（実機の慣性っぽさ）
TILT_GAIN = 0.30  # 速度→傾き角[rad]のゲイン（見た目の演出）

X_LIM, Y_LIM = 1.4, 1.9
Z_MIN, Z_MAX = 0.1, 2.0


class CmdVelSubscriber(Node):
    """/cmd_vel (geometry_msgs/Twist) を購読し、最新の正規化速度指令を保持する。"""

    def __init__(self):
        super().__init__("isaac_tello_sim")
        # [vx, vy, vz, wz]（いずれも正規化 -1〜1 を想定）
        self.cmd = np.zeros(4)
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.get_logger().info("subscribed to /cmd_vel")

    def _cb(self, msg: Twist):
        self.cmd = np.array(
            [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
        )


def main():
    # --- アリーナ + World + Tello役の箱（step1/2 と同じ）-----------------
    assert os.path.exists(ARENA_USD), f"USDが見つからない: {ARENA_USD}"
    print(f"[INFO] opening arena: {ARENA_USD}")
    open_stage(ARENA_USD)
    world = World()
    drone = VisualCuboid(
        prim_path="/World/Tello",
        name="tello",
        position=START_POS,
        scale=np.array([0.18, 0.18, 0.04]),
        color=np.array([0.1, 0.6, 1.0]),
    )
    world.reset()

    # --- ROS2 を初期化して購読ノードを作る ------------------------------
    rclpy.init()
    sub = CmdVelSubscriber()
    print("[INFO] ready. 別端末で teleop_twist_keyboard を起動して /cmd_vel を流してね")

    # --- 機体の状態（step2 と同じ自前積分）-----------------------------
    pos = START_POS.copy().astype(float)
    yaw = 0.0
    vel = np.zeros(3)
    dt = world.get_physics_dt()

    while simulation_app.is_running():
        # 1) 入力源：ROS2 から最新 /cmd_vel を取り込む（ブロックしない）
        rclpy.spin_once(sub, timeout_sec=0.0)
        cmd = sub.cmd
        # 正規化 [-1,1] にクランプ → 実機相当の速度へスケール（機体座標）
        n = np.clip(cmd, -1.0, 1.0)
        cmd_v_body = np.array([n[0] * MAX_VXY, n[1] * MAX_VXY, n[2] * MAX_VZ])
        wz = n[3] * MAX_WZ

        # --- ここから下は step2 と完全に同じ速度モデル -----------------
        # 2) 指令速度へなめらかに追従
        vel += (cmd_v_body - vel) * SMOOTH
        # 3) ヨー更新
        yaw += wz * dt
        # 4) 機体座標→世界座標
        c, s = np.cos(yaw), np.sin(yaw)
        world_v = np.array([
            c * vel[0] - s * vel[1],
            s * vel[0] + c * vel[1],
            vel[2],
        ])
        # 5) 位置を積分してアリーナ内にクランプ
        pos += world_v * dt
        pos[0] = np.clip(pos[0], -X_LIM, X_LIM)
        pos[1] = np.clip(pos[1], -Y_LIM, Y_LIM)
        pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)
        # 6) 傾き演出
        pitch = -TILT_GAIN * vel[0]
        roll = TILT_GAIN * vel[1]
        quat = euler_angles_to_quat(np.array([roll, pitch, yaw]))
        # 7) 反映して1ステップ
        drone.set_world_pose(position=pos, orientation=quat)
        world.step(render=True)

    # --- 後始末 ---------------------------------------------------------
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
