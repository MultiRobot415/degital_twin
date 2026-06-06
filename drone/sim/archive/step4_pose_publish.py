# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フェーズ2 / ステップ4：真値ポーズを publish（実機モーキャプの代役）。

step3 からの追加点は「出力」だけ:
    step3: /cmd_vel を購読して飛ばす
    step4: それに加えて、機体の真値ポーズを publish        ← ここを追加
           - geometry_msgs/PoseStamped を POSE_TOPIC へ
           - TF (WORLD_FRAME -> BODY_FRAME) も broadcast（RViz表示用）

これで「実機の OptiTrack/Vicon が出す位置情報」を、Sim の真値で置き換えたことになる。
→ あとで書く制御ノードは、実機でもSimでも同じ pose トピックを購読すればよい。

実機のモーキャプに合わせたいときは POSE_TOPIC / WORLD_FRAME を実機の値に変えるだけ。

実行方法:
    source /home/utsubo/env_isaaclab/bin/activate
    source /opt/ros/humble/setup.bash
    python ~/research/drone/sim/step4_pose_publish.py

別端末で操作（仮の蛇口）:
    source /opt/ros/humble/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

別端末で確認:
    ros2 topic echo /tello/pose          # 真値ポーズが流れているか
    rviz2                                 # Fixed Frame=world, TF / PoseStamped を追加
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim を起動
# =============================================================================
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Step4: cmd_vel teleop + pose publish")
parser.add_argument("--headless", action="store_true", help="GUIを出さない")
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

# =============================================================================
# 2) ここから core / rclpy を import できる
# =============================================================================
import os

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import open_stage

# --- アリーナ USD ----------------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARENA_USD = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "env", "drone_arena.usd"))

# --- ROS インターフェース設定（実機モーキャプに合わせたいならここを変える）---
POSE_TOPIC = "/tello/pose"   # 真値ポーズを流すトピック（= 実機mocapのトピックに相当）
WORLD_FRAME = "world"        # 基準フレーム（mocapのワールド原点）
BODY_FRAME = "tello"         # 機体フレーム

# --- 動きのパラメータ（step2/3 と同じ）-------------------------------------
START_POS = np.array([0.0, 0.0, 1.0])
MAX_VXY = 1.0
MAX_VZ = 0.6
MAX_WZ = 1.2
SMOOTH = 0.1
TILT_GAIN = 0.30
X_LIM, Y_LIM = 1.4, 1.9
Z_MIN, Z_MAX = 0.1, 2.0


class TelloSimNode(Node):
    """/cmd_vel を購読し、真値ポーズ(PoseStamped + TF)を publish するノード。"""

    def __init__(self):
        super().__init__("isaac_tello_sim")
        self.cmd = np.zeros(4)  # [vx, vy, vz, wz]（正規化 -1〜1）
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"sub:/cmd_vel  pub:{POSE_TOPIC} (+TF {WORLD_FRAME}->{BODY_FRAME})")

    def _cb(self, msg: Twist):
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])

    def publish_pose(self, pos: np.ndarray, quat_wxyz: np.ndarray):
        """真値ポーズを PoseStamped と TF の両方で流す。
        quat_wxyz は (w,x,y,z) 順。ROS のメッセージは (x,y,z,w) 順なので詰め替える。
        """
        now = self.get_clock().now().to_msg()
        w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])

        # --- PoseStamped（実機mocapと同じ型）---
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = WORLD_FRAME
        ps.pose.position.x = float(pos[0])
        ps.pose.position.y = float(pos[1])
        ps.pose.position.z = float(pos[2])
        ps.pose.orientation.x = x
        ps.pose.orientation.y = y
        ps.pose.orientation.z = z
        ps.pose.orientation.w = w
        self.pose_pub.publish(ps)

        # --- TF（RViz で機体フレームを動かして見るため）---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = WORLD_FRAME
        t.child_frame_id = BODY_FRAME
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(t)


def main():
    # --- アリーナ + World + Tello役の箱 ----------------------------------
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

    # --- ROS2 ノード -----------------------------------------------------
    rclpy.init()
    node = TelloSimNode()
    print("[INFO] ready. teleop で /cmd_vel を流し、ros2 topic echo /tello/pose で確認できる")

    # --- 機体の状態（step2/3 と同じ自前積分）---------------------------
    pos = START_POS.copy().astype(float)
    yaw = 0.0
    vel = np.zeros(3)
    dt = world.get_physics_dt()

    while simulation_app.is_running():
        # 1) 入力：/cmd_vel を取り込む
        rclpy.spin_once(node, timeout_sec=0.0)
        n = np.clip(node.cmd, -1.0, 1.0)
        cmd_v_body = np.array([n[0] * MAX_VXY, n[1] * MAX_VXY, n[2] * MAX_VZ])
        wz = n[3] * MAX_WZ

        # 2-5) 速度モデル（step2/3 と同一）
        vel += (cmd_v_body - vel) * SMOOTH
        yaw += wz * dt
        c, s = np.cos(yaw), np.sin(yaw)
        world_v = np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]])
        pos += world_v * dt
        pos[0] = np.clip(pos[0], -X_LIM, X_LIM)
        pos[1] = np.clip(pos[1], -Y_LIM, Y_LIM)
        pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)

        # 6) 傾き演出
        pitch = -TILT_GAIN * vel[0]
        roll = TILT_GAIN * vel[1]
        quat = euler_angles_to_quat(np.array([roll, pitch, yaw]))  # (w,x,y,z)

        # 7) Sim に反映 + 真値ポーズを publish（← step4 の追加点）
        drone.set_world_pose(position=pos, orientation=quat)
        node.publish_pose(pos, quat)
        world.step(render=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
