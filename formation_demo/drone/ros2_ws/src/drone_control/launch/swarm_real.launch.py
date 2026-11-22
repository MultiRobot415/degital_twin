# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機用インフラ（複数機）: 起動しても“何も飛ばない”土台だけを立てる。

    mocap_driver        OptiTrack(Motive) → /rigid_bodies（lifecycle・自動 configure→activate）
    pose_bridge × N     /rigid_bodies → /<name>/pose（剛体ID選択＋座標変換。convert_y_up は swarm.yaml）
    tello_swarm_driver  全機と 1 ソケットで通信（/<name>/cmd_vel, /<name>/tello_action, /<name>/flight_data）
    RViz                全機の pose / 目標 / 基準の可視化・2D Goal Pose

機体を動かすのはこの上で（別端末）:
    ros2 launch drone_control formation.launch.py formation:=reference_tracking
    ros2 run drone_control formation_commander     # t=離陸 g=開始 … h=停止 l=着陸

使い方:
    source /opt/ros/humble/setup.bash
    cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control swarm_real.launch.py

引数:
    swarm_file:=<path>    機体設定（IP・剛体ID・convert_y_up）
    config_file:=<path>   mocap driver の params.yaml
    use_rviz:=false       RViz を起動しない
"""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg

from drone_control import swarm_config


def _swarm_nodes(context):
    swarm_file = LaunchConfiguration("swarm_file").perform(context)
    swarm = swarm_config.load_swarm(swarm_file)
    bridges = [
        Node(
            package="drone_control",
            executable="pose_bridge",
            name=f"pose_bridge_{d.name}",
            output="screen",
            parameters=[{
                "rigid_body_id": str(d.rigid_body_id),
                "convert_y_up": swarm.convert_y_up,
                "output_topic": f"/{d.name}/pose",
                "world_frame": swarm.world_frame,
                "body_frame": d.name,
            }],
        )
        for d in swarm.drones
    ]
    driver = Node(
        package="drone_control",
        executable="tello_swarm_driver",
        name="tello_swarm_driver",
        output="screen",
        parameters=[{"swarm_file": swarm_file}],
    )
    return bridges + [driver]


def generate_launch_description():
    rviz_config = os.path.join(swarm_config.share_dir(), "rviz", "swarm.rviz")
    default_mocap_params = os.path.join(
        get_package_share_directory("mocap4r2_optitrack_driver"),
        "config",
        "mocap4r2_optitrack_driver_params.yaml",
    )

    # --- mocap ドライバ（ライフサイクルノード）→ /rigid_bodies。bringup_real.launch.py と同じ ---
    mocap_driver = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        executable="mocap4r2_optitrack_driver_main",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )
    mocap_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
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
        DeclareLaunchArgument("swarm_file", default_value=swarm_config.default_swarm_file()),
        DeclareLaunchArgument("config_file", default_value=default_mocap_params),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        mocap_driver,
        mocap_configure,
        mocap_activate_on_inactive,

        OpaqueFunction(function=_swarm_nodes),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
