# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フォーメーション制御（formation_agent × 機体数）を起動する。Sim と実機で共通。

先にインフラを起動しておく（どちらか一方）:
    Sim : ros2 launch drone_control swarm_sim.launch.py
    実機: ros2 launch drone_control swarm_real.launch.py

このlaunch:
    ros2 launch drone_control formation.launch.py formation:=leader_follower

操作（別端末）:
    ros2 run drone_control formation_commander     # t=離陸 g=開始 w/a/s/d/r/f=基準移動 h=停止 l=着陸

agent は /formation/enable=true になるまで cmd_vel を出さない（起動しただけでは動かない）。

引数:
    formation:=reference_tracking | leader_follower | consensus   … formation_agent（変位ベース）
               ch3_net1 | ch3_net2                                … consensus_agent（論文 第3章・law: consensus_ch3）
               drone_show                                         … show_agent（ルールベースのショー・law: drone_show）
               <yaml へのパス>
    swarm_file:=<path>   機体設定（既定: share/drone_control/config/swarm.yaml）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_control import swarm_config


def _agents(context):
    swarm_file = LaunchConfiguration("swarm_file").perform(context)
    formation = LaunchConfiguration("formation").perform(context)
    formation_path = swarm_config.formation_file(formation)
    # 起動前にここで検証する（不正な設定・基準に届かないグラフはこの時点でエラー）
    swarm = swarm_config.load_swarm(swarm_file)
    law = swarm_config.formation_law(formation_path)
    if law == swarm_config.CONSENSUS_CH3:
        swarm_config.load_consensus(formation_path, swarm)
        executable = "consensus_agent"
    elif law == swarm_config.DRONE_SHOW:
        swarm_config.load_drone_show(formation_path, swarm)
        executable = "show_agent"
    else:
        swarm_config.load_formation(formation_path, swarm)
        executable = "formation_agent"
    return [
        Node(
            package="drone_control",
            executable=executable,
            name=f"{executable}_{name}",
            output="screen",
            parameters=[{"name": name, "swarm_file": swarm_file, "formation": formation_path}],
        )
        for name in swarm.names
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("swarm_file", default_value=swarm_config.default_swarm_file()),
        DeclareLaunchArgument("formation", default_value="reference_tracking"),
        OpaqueFunction(function=_agents),
    ])
