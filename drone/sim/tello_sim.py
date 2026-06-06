# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Tello デジタルツインのシミュレータ本体（Isaac Sim 側）。

URDF変換した tello.usd を「ダイナミック剛体」として読み込み、ROS2 の /cmd_vel を
速度指令として物理エンジンに動かしてもらう。機体の真値ポーズを /tello/pose で publish。

役割分担:
    このスクリプト（Isaac Sim / venv で起動）= 仮想の機体＋世界
    ros2_ws の position_controller / teleop       = 制御ロジック（実機と共通）
    → 同じ ROS2 トピックでつながり、実機と差し替え可能なデジタルツインになる。

設計（B1: 剛体＋速度制御）:
    - 重力オフ（Tello は放っておいてもホバリングする実機の事実に合わせる）
    - 機体座標の速度指令 → 現在のヨーで世界座標に回す → 剛体の linear velocity に設定
    - ヨーは angular velocity (z) で回す
    - 衝突オン：地面など当たり判定のある物体に物理的に阻まれる
    - pose は「物理から読み戻した実際の姿勢」を publish（より本物の真値）

発展（後で）: 重力をオンにし、上下のホバリング制御 → 推力/トルク と足していけば B2 になる。

実行方法:
    source /home/utsubo/env_isaaclab/bin/activate
    source /opt/ros/humble/setup.bash
    python ~/research/drone/sim/tello_sim.py
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim を起動
# =============================================================================
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Tello digital-twin simulator (Isaac Sim)")
parser.add_argument("--headless", action="store_true", help="GUIを出さない")
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

# =============================================================================
# 2) ここから core / rclpy / pxr を import できる
# =============================================================================
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from pxr import Usd, UsdPhysics, PhysxSchema

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage, get_current_stage

# --- ファイル/トピック設定 -------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARENA_USD = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "env", "drone_arena.usd"))
TELLO_USD = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "robots", "tello.usd"))
TELLO_PRIM = "/World/Tello"

POSE_TOPIC = "/tello/pose"
WORLD_FRAME = "world"
BODY_FRAME = "tello"

# --- 動きのパラメータ -------------------------------------------------------
START_POS = np.array([0.0, 0.0, 1.0])
MAX_VXY = 1.0
MAX_VZ = 0.6
MAX_WZ = 1.2
SMOOTH = 0.1
X_LIM, Y_LIM = 1.4, 1.9
Z_MIN, Z_MAX = 0.1, 2.0


def yaw_from_quat(w, x, y, z) -> float:
    """クォータニオン(w,x,y,z) → ヨー角[rad]。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def setup_tello_rigidbody() -> SingleRigidPrim:
    """tello.usd を参照し、剛体primを自動発見して「重力オフの自由剛体」に整える。"""
    add_reference_to_stage(usd_path=TELLO_USD, prim_path=TELLO_PRIM)
    stage = get_current_stage()

    rb_path = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(TELLO_PRIM)):
        # 単一リンクなので articulation 化されていれば解除して素直な剛体にする
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb_path = prim.GetPath().pathString
            # 重力オフ（PhysX 拡張スキーマで設定）
            physx = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx.CreateDisableGravityAttr(True)

    assert rb_path is not None, "tello.usd の中に RigidBody が見つからない"
    print(f"[INFO] rigid body prim: {rb_path}")
    return SingleRigidPrim(prim_path=rb_path, name="tello")


class TelloSimNode(Node):
    """/cmd_vel を購読し、真値ポーズ(PoseStamped + TF)を publish するノード。"""

    def __init__(self):
        super().__init__("isaac_tello_sim")
        self.cmd = np.zeros(4)
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"sub:/cmd_vel  pub:{POSE_TOPIC} (+TF {WORLD_FRAME}->{BODY_FRAME})")

    def _cb(self, msg: Twist):
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])

    def publish_pose(self, pos: np.ndarray, quat_wxyz: np.ndarray):
        now = self.get_clock().now().to_msg()
        w, x, y, z = (float(quat_wxyz[0]), float(quat_wxyz[1]),
                      float(quat_wxyz[2]), float(quat_wxyz[3]))

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = WORLD_FRAME
        ps.pose.position.x = float(pos[0])
        ps.pose.position.y = float(pos[1])
        ps.pose.position.z = float(pos[2])
        ps.pose.orientation.x, ps.pose.orientation.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = z, w
        self.pose_pub.publish(ps)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = WORLD_FRAME
        t.child_frame_id = BODY_FRAME
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x, t.transform.rotation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = z, w
        self.tf_broadcaster.sendTransform(t)


def main():
    # --- アリーナ + World + 剛体Tello ------------------------------------
    assert os.path.exists(ARENA_USD), f"アリーナUSDが無い: {ARENA_USD}"
    assert os.path.exists(TELLO_USD), f"TelloのUSDが無い（先に convert_tello_urdf.py を実行）: {TELLO_USD}"
    print(f"[INFO] opening arena: {ARENA_USD}")
    open_stage(ARENA_USD)
    world = World()
    tello = setup_tello_rigidbody()

    world.reset()  # 物理が再生状態になる

    # 初期位置にセットして静止させる
    tello.set_world_pose(position=START_POS)
    tello.set_linear_velocity(np.zeros(3))
    tello.set_angular_velocity(np.zeros(3))

    rclpy.init()
    node = TelloSimNode()
    print("[INFO] ready (rigid body, gravity OFF, velocity-controlled).")

    vel = np.zeros(3)  # なめらか追従用の現在速度（機体座標）

    while simulation_app.is_running():
        # 0) 物理が今いる場所・向きを読む（速度の向き計算と publish に使う）
        pos, quat = tello.get_world_pose()  # quat は (w,x,y,z)
        yaw = yaw_from_quat(quat[0], quat[1], quat[2], quat[3])

        # 1) 入力：/cmd_vel
        rclpy.spin_once(node, timeout_sec=0.0)
        n = np.clip(node.cmd, -1.0, 1.0)
        cmd_v_body = np.array([n[0] * MAX_VXY, n[1] * MAX_VXY, n[2] * MAX_VZ])
        wz = n[3] * MAX_WZ

        # 2) なめらかに追従（実機の慣性っぽさ）
        vel += (cmd_v_body - vel) * SMOOTH

        # 3) 機体座標 → 世界座標（現在のヨーで回す）
        c, s = math.cos(yaw), math.sin(yaw)
        world_v = np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]])

        # 4) アリーナ外へ出ようとする速度成分は止める（壁/ネットの当たり判定はまだ無いので）
        if (pos[0] >= X_LIM and world_v[0] > 0) or (pos[0] <= -X_LIM and world_v[0] < 0):
            world_v[0] = 0.0
        if (pos[1] >= Y_LIM and world_v[1] > 0) or (pos[1] <= -Y_LIM and world_v[1] < 0):
            world_v[1] = 0.0
        if (pos[2] >= Z_MAX and world_v[2] > 0) or (pos[2] <= Z_MIN and world_v[2] < 0):
            world_v[2] = 0.0

        # 5) 剛体に速度を与える（テレポートせず、物理に動かしてもらう）
        tello.set_linear_velocity(world_v)
        tello.set_angular_velocity(np.array([0.0, 0.0, wz]))

        # 6) pose を publish（物理から読んだ実際の姿勢）
        node.publish_pose(pos, quat)

        world.step(render=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
