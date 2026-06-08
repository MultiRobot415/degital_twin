# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機用 一括起動 launch：OptiTrack(mocap) → 制御 を1コマンドで立てる。

Sim の bringup.launch.py の「実機版」。pose の出どころが Sim ではなく
OptiTrack ドライバになる。デジタルツインの設計どおり、中央の position_controller
は無改造でそのまま使う。

立ち上がる構成:
    mocap4r2_optitrack_driver_node (LifecycleNode)
        │  ← configure→activate を自動でかける（手動 lifecycle set は不要）
        ▼
    /rigid_bodies (mocap4r2_msgs/RigidBodies)
        │
        ▼  pose_bridge : 対象IDを選び Z-up に直し /tello/pose へ
    /tello/pose (PoseStamped)
        │
        ▼  position_controller : 目標へ /cmd_vel
    /cmd_vel  → （別途）tello_driver で実機へ

使い方:
    source /opt/ros/humble/setup.bash
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control tello_real.launch.py rigid_body_id:=1 convert_y_up:=false

事前に config の server_address(MotivePCのIP)/local_address(このPCのIP) を
実環境に合わせること（driver の params.yaml、または config_file 引数で差し替え）。

引数:
    rigid_body_id:=1      Motive の Streaming ID（pose_bridge が選ぶ剛体）
    convert_y_up:=false   false=Motive側でZ-up配信 / true=ここでY-up→Z-up変換
    config_file:=<path>   driver の params.yaml を差し替え
    use_rviz:=false       RViz を起動しない
    goal_x/goal_y/goal_z  初期目標位置
"""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg


def generate_launch_description():
    drone_share = get_package_share_directory("drone_control")
    rviz_config = os.path.join(drone_share, "rviz", "tello.rviz")
    default_driver_params = os.path.join(
        get_package_share_directory("mocap4r2_optitrack_driver"),
        "config",
        "mocap4r2_optitrack_driver_params.yaml",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    rigid_body_id = LaunchConfiguration("rigid_body_id")
    convert_y_up = LaunchConfiguration("convert_y_up")
    config_file = LaunchConfiguration("config_file")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")

    # --- mocap ドライバ（ライフサイクルノード）---
    driver_node = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        executable="mocap4r2_optitrack_driver_main",
        output="screen",
        parameters=[config_file],
    )

    # 起動直後に configure をかける（unconfigured -> inactive）
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # inactive に到達したら activate をかける（inactive -> active）= 自動起動
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rigid_body_id", default_value="1"),
        DeclareLaunchArgument("convert_y_up", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_driver_params),
        DeclareLaunchArgument("goal_x", default_value="1.0"),
        DeclareLaunchArgument("goal_y", default_value="1.0"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),

        driver_node,
        configure_event,
        activate_on_inactive,

        # mocap → /tello/pose 橋渡し
        Node(
            package="drone_control",
            executable="pose_bridge",
            name="pose_bridge",
            output="screen",
            parameters=[{
                "rigid_body_id": ParameterValue(rigid_body_id, value_type=str),
                "convert_y_up": ParameterValue(convert_y_up, value_type=bool),
            }],
        ),

        # 位置制御（Sim と共通・無改造）
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

        # RViz（pose/TF 可視化・2D Goal Pose）
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
