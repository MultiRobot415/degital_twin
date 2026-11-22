# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Sim 用インフラ: 実機インフラ（swarm_real.launch.py）の代わりに swarm_sim を立てる。

実機インフラと同じトピック・サービス（/<name>/pose, /<name>/cmd_vel, /<name>/tello_action ...）を出すので、
この上で formation.launch.py / formation_commander をそのまま使える。起動しただけでは飛ばない（全機 LANDED）。

    ros2 launch drone_control swarm_sim.launch.py
    ros2 launch drone_control formation.launch.py formation:=leader_follower   # 別端末
    ros2 run drone_control formation_commander                                 # 別端末

引数:
    emulate_mocap:=true   swarm_sim が /rigid_bodies を Motive 座標で出し、実機と同じ pose_bridge が
                          /<name>/pose に変換する（座標変換の経路まで Sim で検証する）
    use_rviz:=false       RViz を起動しない
    swarm_file:=<path>    機体設定
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_control import swarm_config


def _infra(context):
    swarm_file = LaunchConfiguration("swarm_file").perform(context)
    emulate_mocap = LaunchConfiguration("emulate_mocap").perform(context).lower() in ("true", "1")
    swarm = swarm_config.load_swarm(swarm_file)

    actions = [
        Node(
            package="drone_control",
            executable="swarm_sim",
            name="swarm_sim",
            output="screen",
            parameters=[{"swarm_file": swarm_file, "emulate_mocap": emulate_mocap}],
        )
    ]
    if emulate_mocap:
        actions += _pose_bridges(swarm)
    return actions


def _pose_bridges(swarm):
    """実機と同じ pose_bridge を機体数だけ（swarm_real.launch.py と同じ設定）。"""
    return [
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


def generate_launch_description():
    rviz_config = os.path.join(swarm_config.share_dir(), "rviz", "swarm.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("swarm_file", default_value=swarm_config.default_swarm_file()),
        DeclareLaunchArgument("emulate_mocap", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_infra),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
