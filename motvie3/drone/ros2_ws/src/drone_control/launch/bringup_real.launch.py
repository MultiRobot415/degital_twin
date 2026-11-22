# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機用 インフラ launch：起動しても“何も飛ばない”土台だけを立てる。

設計方針（重要）:
    /cmd_vel を出すノード（position_controller / keyboard_teleop）は**ここに入れない**。
    この launch が立ち上げるのは「電源を入れても機体が動かない」インフラだけ:

        natnet_receiver OptiTrack(Motive) → /rigid_bodies（受信だけ。mocap_source:=sdk で旧ドライバ）
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
    cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control bringup_real.launch.py \\
        rigid_body_id:=<MotiveのID> drone_ip:=192.168.11.51

引数:
    drone_ip:=192.168.11.51   実機Telloの(stationモード時の)IP
    rigid_body_id:=1          Motive の Streaming ID（pose_bridge が選ぶ剛体）
    convert_y_up:=true        true=Motive側がY-up配信(この研究室の設定)→Z-up変換 / false=素通し
    mocap_source:=natnet      natnet（既定・Motive 3.x）/ sdk（旧 mocap4r2 ドライバ）
    natnet_version:=4.1       Motive 3.x = 4.1 / Motive 1.x = 2.10
    config_file:=<path>       mocap driver の params.yaml を差し替え（mocap_source:=sdk のとき）
    use_rviz:=false           RViz を起動しない
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from drone_control.mocap_launch import mocap_actions


def generate_launch_description():
    drone_share = get_package_share_directory("drone_control")
    rviz_config = os.path.join(drone_share, "rviz", "tello.rviz")
    use_rviz = LaunchConfiguration("use_rviz")
    drone_ip = LaunchConfiguration("drone_ip")
    rigid_body_id = LaunchConfiguration("rigid_body_id")
    convert_y_up = LaunchConfiguration("convert_y_up")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("drone_ip", default_value="192.168.11.51"),
        DeclareLaunchArgument("rigid_body_id", default_value="1"),
        DeclareLaunchArgument("convert_y_up", default_value="true"),

        # mocap: Motive → /rigid_bodies（mocap_source で natnet_receiver / mocap4r2 ドライバを選ぶ）
        *mocap_actions(),

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
