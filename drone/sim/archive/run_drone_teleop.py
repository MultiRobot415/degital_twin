# Copyright (c) 2025
# SPDX-License-Identifier: BSD-3-Clause
"""Crazyflie (quadcopter) teleop bridge for Isaac Sim + Isaac Lab.

このスクリプトは「Isaac Sim を起動するプロセス」です。
中で rclpy ノードを同居させ、別プロセスの teleop が流す /cmd_vel を購読して
機体に推力(thrust)とモーメント(moment)を加えます。

実行方法（colcon ではなく venv の python で直接！）:

    source /home/utsubo/env_isaaclab/bin/activate
    source /opt/ros/humble/setup.bash
    python ~/research/drone/sim/run_drone_teleop.py

別端末から操作:

    source /opt/ros/humble/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

オプション:
    --no_ros        ROS を使わずホバリングだけ（Step1の動作確認用）
    --headless      GUI を出さない
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim アプリを起動する。
#    これより前に isaaclab / carb / isaacsim を import してはいけない。
# =============================================================================
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Crazyflie ROS2 teleop bridge")
parser.add_argument("--no_ros", action="store_true", help="ROSを使わずホバリングのみ")
AppLauncher.add_app_launcher_args(parser)  # --headless や --device 等を自動追加
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# =============================================================================
# 2) ここから下でだけ isaaclab / isaacsim を import できる
# =============================================================================
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

from isaaclab_assets.robots.quadcopter import CRAZYFLIE_CFG  # noqa: E402


# -----------------------------------------------------------------------------
# 制御パラメータ（いじって挙動を学ぶ場所）
# -----------------------------------------------------------------------------
GRAVITY = 9.81
K_THRUST = 0.5    # linear.z -> 上下の追加推力ゲイン
K_PITCH = 0.02    # linear.x -> ピッチ方向モーメント（前後）
K_ROLL = 0.02     # linear.y -> ロール方向モーメント（左右）
K_YAW = 0.01      # angular.z -> ヨー方向モーメント（旋回）


def design_scene() -> Articulation:
    """地面・ライト・Crazyflie をスポーンしてロボットを返す。"""
    # 地面
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    # ライト
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)
    # ドローン本体（単体なので prim_path を固定パスに上書き）
    robot_cfg = CRAZYFLIE_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"
    robot = Articulation(cfg=robot_cfg)
    return robot


class CmdVelSubscriber:
    """/cmd_vel (geometry_msgs/Twist) を購読して最新値を保持する rclpy ノード。

    ROS が無効(--no_ros)のときは何もしないダミーとして振る舞う。
    """

    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.linear = [0.0, 0.0, 0.0]   # x, y, z
        self.angular = [0.0, 0.0, 0.0]  # x, y, z
        if not enabled:
            return
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        rclpy.init()
        self._rclpy = rclpy
        self._node = Node("drone_teleop_bridge")
        self._node.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self._node.get_logger().info("Subscribed to /cmd_vel")

    def _cb(self, msg):
        self.linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.angular = [msg.angular.x, msg.angular.y, msg.angular.z]

    def spin_once(self):
        """ブロックせずに溜まったメッセージを1回処理する。"""
        if self.enabled:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def shutdown(self):
        if self.enabled:
            self._node.destroy_node()
            self._rclpy.shutdown()


def run_simulator(sim: SimulationContext, robot: Articulation, cmd: CmdVelSubscriber):
    sim_dt = sim.get_physics_dt()

    # 機体本体のボディindexと総質量（ホバリングに必要な推力 = 質量 * g）
    body_id = robot.find_bodies("body")[0]
    mass = robot.root_physx_view.get_masses()[0].sum().item()
    hover_thrust = mass * GRAVITY
    print(f"[INFO] mass={mass:.5f} kg, hover_thrust={hover_thrust:.5f} N")

    # set_external_force_and_torque に渡すバッファ: 形状 (num_instances, num_bodies, 3)
    thrust = torch.zeros(1, 1, 3, device=sim.device)
    moment = torch.zeros(1, 1, 3, device=sim.device)

    count = 0
    while simulation_app.is_running():
        # 定期的に初期姿勢へリセット（墜落しても復帰できるように）
        if count % 2000 == 0:
            count = 0
            root_state = robot.data.default_root_state.clone()
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])
            robot.reset()
            print("[INFO] reset drone state")

        # ROS から最新の cmd_vel を取得
        cmd.spin_once()

        # cmd_vel -> 推力 / モーメント へのマッピング
        #   linear.z  : 上下（基本ホバリング推力に加算）
        #   linear.x  : 前後（ピッチモーメント）  ※簡易モデル
        #   linear.y  : 左右（ロールモーメント）  ※簡易モデル
        #   angular.z : 旋回（ヨーモーメント）
        thrust[0, 0, 2] = hover_thrust + K_THRUST * cmd.linear[2]
        moment[0, 0, 0] = K_ROLL * cmd.linear[1]
        moment[0, 0, 1] = K_PITCH * cmd.linear[0]
        moment[0, 0, 2] = K_YAW * cmd.angular[2]

        robot.set_external_force_and_torque(thrust, moment, body_ids=body_id)
        robot.write_data_to_sim()
        sim.step()
        count += 1
        robot.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.5])

    robot = design_scene()
    sim.reset()  # ここで物理が「再生」状態になる
    print("[INFO] setup complete")

    cmd = CmdVelSubscriber(enabled=not args_cli.no_ros)
    try:
        run_simulator(sim, robot, cmd)
    finally:
        cmd.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
