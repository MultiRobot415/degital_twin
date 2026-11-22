# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機 launch（swarm_real / bringup_real）共通: Motive → /rigid_bodies を出すノードを選んで立てる。

    mocap_source:=natnet（既定）  natnet_receiver（受信だけ。Motive 3.x はこちら）
                                   natnet_version:=4.1（Motive 3.x）/ 2.10（Motive 1.x）
                                   local_address:=192.168.11.19（マルチキャストを受ける NIC の IP）
    mocap_source:=sdk             mocap4r2_optitrack_driver（NatNet SDK 3.0。Motive のコマンドポート
                                   1510 から返事が来ないと何も出さない。config_file:=<params.yaml>）

どちらも /rigid_bodies に同じ形で出すので、pose_bridge 以降は変わらない。
違いは drone/docs/motive3_natnet.md。
"""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg


def mocap_actions():
    """launch 引数の宣言と、mocap_source に応じた /rigid_bodies の出し手を返す。"""
    default_sdk_params = os.path.join(
        get_package_share_directory("mocap4r2_optitrack_driver"),
        "config",
        "mocap4r2_optitrack_driver_params.yaml",
    )
    use_natnet = LaunchConfigurationEquals("mocap_source", "natnet")
    use_sdk = LaunchConfigurationEquals("mocap_source", "sdk")

    receiver = Node(
        package="drone_control",
        executable="natnet_receiver",
        name="natnet_receiver",
        output="screen",
        parameters=[{
            "natnet_version": ParameterValue(LaunchConfiguration("natnet_version"), value_type=str),
            "local_address": ParameterValue(LaunchConfiguration("local_address"), value_type=str),
        }],
        condition=use_natnet,
    )

    # --- 旧: mocap4r2 ドライバ（ライフサイクルノード・自動 configure→activate）---
    sdk_driver = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        executable="mocap4r2_optitrack_driver_main",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        condition=use_sdk,
    )
    sdk_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(sdk_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=use_sdk,
    )
    sdk_activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sdk_driver,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matchers.matches_action(sdk_driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=use_sdk,
    )

    return [
        DeclareLaunchArgument("mocap_source", default_value="natnet", choices=["natnet", "sdk"]),
        DeclareLaunchArgument("natnet_version", default_value="4.1"),
        DeclareLaunchArgument("local_address", default_value="192.168.11.19"),
        DeclareLaunchArgument("config_file", default_value=default_sdk_params),
        receiver,
        sdk_driver,
        sdk_configure,
        sdk_activate_on_inactive,
    ]
