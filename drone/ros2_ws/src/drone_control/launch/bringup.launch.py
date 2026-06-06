# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""制御ノード（＋RViz）を一括起動する launch ファイル。

使い方:
    # 端末①（先に）：Isaac Sim 側のシミュレータを起動（別env・venv）
    source /home/utsubo/env_isaaclab/bin/activate
    source /opt/ros/humble/setup.bash
    python ~/research/drone/sim/tello_sim.py

    # 端末②：このlaunchで制御＋RVizを起動
    source /opt/ros/humble/setup.bash
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control bringup.launch.py

    → RViz の "2D Goal Pose" でフィールドをクリックすると、その位置へ飛ぶ。

引数:
    use_rviz:=false           RVizを起動しない
    goal_x:=1.0 goal_y:=-0.5 goal_z:=1.3   初期目標位置
"""

from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_control")
    rviz_config = os.path.join(pkg_share, "rviz", "tello.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("goal_x", default_value="1.0"),
        DeclareLaunchArgument("goal_y", default_value="1.0"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),

        # 位置制御ノード（pose -> /cmd_vel）
        Node(
            package="drone_control",
            executable="position_controller",
            name="position_controller",
            output="screen",
            parameters=[{
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_z": ParameterValue(goal_z, value_type=float),
            }],
        ),

        # RViz（真値ポーズ・TF の可視化＋2D Goal Pose で目標指定）
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
