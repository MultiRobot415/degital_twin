# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機用 インフラ launch：起動しても“何も飛ばない”土台だけを立てる。

設計方針（重要）:
    /cmd_vel を出すノード（position_controller / keyboard_teleop）は**ここに入れない**。
    この launch が立ち上げるのは「電源を入れても機体が動かない」インフラだけ:

        mocap_driver   OptiTrack(Motive) → /rigid_bodies（lifecycle・自動activate）
        pose_bridge    /rigid_bodies → /tello/pose（対象ID選択＋Z-up変換）
        tello_driver   実機Telloと喋る通訳（/cmd_vel受け・/tello_action提供・/flight_data出し）
        RViz           pose/TF 可視化・2D Goal Pose

    機体を動かすのは、この土台の上で**明示的に1つだけ** ros2 run する:
        ros2 run drone_control keyboard_teleop      # 手動（T=離陸 / L=着陸 内蔵）
        ros2 run drone_control position_controller  # 自動（目標追従・起動時は現在地ホールド）
    ※ teleop と position_controller は同時に動かさない（どちらも /cmd_vel に書いて競合）。
       離着陸は /tello_action サービスなので /cmd_vel とは競合しない。

使い方:
    source /opt/ros/humble/setup.bash
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control bringup_real.launch.py \\
        rigid_body_id:=<MotiveのID> drone_ip:=192.168.11.51

引数:
    drone_ip:=192.168.11.51   実機Telloの(stationモード時の)IP
    rigid_body_id:=1          Motive の Streaming ID（pose_bridge が選ぶ剛体）
    convert_y_up:=true        true=Motive側がY-up配信(この研究室の設定)→Z-up変換 / false=素通し
    config_file:=<path>       mocap driver の params.yaml を差し替え
    use_rviz:=false           RViz を起動しない
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
    default_mocap_params = os.path.join(
        get_package_share_directory("mocap4r2_optitrack_driver"),
        "config",
        "mocap4r2_optitrack_driver_params.yaml",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    drone_ip = LaunchConfiguration("drone_ip")
    rigid_body_id = LaunchConfiguration("rigid_body_id")
    convert_y_up = LaunchConfiguration("convert_y_up")
    config_file = LaunchConfiguration("config_file")

    # --- mocap ドライバ（ライフサイクルノード）→ /rigid_bodies ---
    mocap_driver = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        executable="mocap4r2_optitrack_driver_main",
        output="screen",
        parameters=[config_file],
    )
    # 起動直後に configure（unconfigured -> inactive）
    mocap_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    # inactive 到達で activate（inactive -> active）= 自動起動
    mocap_activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mocap_driver,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("drone_ip", default_value="192.168.11.51"),
        DeclareLaunchArgument("rigid_body_id", default_value="1"),
        DeclareLaunchArgument("convert_y_up", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_mocap_params),

        # mocap: OptiTrack → /rigid_bodies（自動 configure→activate）
        mocap_driver,
        mocap_configure,
        mocap_activate_on_inactive,

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

        # 実機Telloと喋る通訳（/cmd_vel受け・/tello_action提供・/flight_data出し）
        Node(
            package="tello_driver",
            executable="tello_driver_main",
            name="tello_driver",
            output="screen",
            parameters=[{
                "drone_ip": ParameterValue(drone_ip, value_type=str),
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
