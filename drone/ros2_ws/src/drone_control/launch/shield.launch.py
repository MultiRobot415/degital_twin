# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""SHIELD/CBF 実験レイヤ launch：障害物ブリッジ＋安全フィルタをまとめて立てる。

bringup_real.launch.py（インフラ：mocap/pose_bridge/tello_driver/RViz）の**上に**重ねて使う。
「障害物環境下で CBF と SHIELD の挙動を比べる」ときに毎回セットで要る2ノードを1コマンドに:

    obstacle_bridge       /rigid_bodies(ID) → /shield/obstacles_state（mocap障害物）
    shield_filter_node    /cmd_vel_nom を安全な /cmd_vel に整形（mode で CBF/SHIELD 切替）

■ teleop は含めない（意図的）
    keyboard_teleop は termios で生キー入力を専有するため launch ノードに混ぜると壊れる。
    従来どおり**別ターミナルで手動**起動する（生指令を /cmd_vel_nom へリマップ）:
        ros2 run drone_control keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_nom

■ shield_filter_node は torch が要る → venv python で起動する
    `ros2 run` は system python を使い torch が無いので不可。ここでは python_bin
    （既定 ~/env_isaaclab/bin/python）で `-m dist_learn.shield_filter_node` を直起動する。
    そのため **`ros2 launch` する前に必ず ROS と install を source** しておくこと
    （venv python が dist_learn を import するための PYTHONPATH を引き継ぐ）:
        source /opt/ros/humble/setup.bash
        cd ~/research/drone/ros2_ws && source install/setup.bash

使い方（CBF と SHIELD は mode だけで切替）:
    # SHIELD（CVAE 確率バリア）
    ros2 launch drone_control shield.launch.py mode:=shield \
        obstacle_ids:="[2]" obstacle_radii:="[0.15]"
    # CBF（素の DTCBF・CVAE 不使用）
    ros2 launch drone_control shield.launch.py mode:=nominal

引数:
    mode:=shield|nominal      shield=CVAE確率バリア / nominal=素のCBF（既定 shield）
    obstacle_ids:="[2]"       障害物にする Motive Streaming ID のリスト
    obstacle_radii:="[0.15]"  各IDの物理半径[m]（obstacle_ids と同じ長さ）
    convert_y_up:=true        bringup と必ず一致させる（研究室設定は true）
    robot_radius:=0.15        安全境界 R = 障害物半径 + robot_radius に効く
    u_max:=0.5                フィルタ出力の上限[m/s]
    python_bin:=<path>        torch入り python（既定 ~/env_isaaclab/bin/python）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    obstacle_ids = LaunchConfiguration("obstacle_ids")
    obstacle_radii = LaunchConfiguration("obstacle_radii")
    convert_y_up = LaunchConfiguration("convert_y_up")
    robot_radius = LaunchConfiguration("robot_radius")
    u_max = LaunchConfiguration("u_max")
    python_bin = LaunchConfiguration("python_bin")

    default_python = os.path.expanduser("~/env_isaaclab/bin/python")

    # 障害物ブリッジ：ros2 run と同じ形で起動（リスト引数を ros2 引数パーサに任せる）
    obstacle_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "drone_control", "obstacle_bridge", "--ros-args",
            "-p", ["obstacle_ids:=", obstacle_ids],
            "-p", ["obstacle_radii:=", obstacle_radii],
            "-p", ["convert_y_up:=", convert_y_up],
        ],
        output="screen",
    )

    # 安全フィルタ：torch が要るので venv python で -m 起動
    shield_filter = ExecuteProcess(
        cmd=[
            python_bin, "-m", "dist_learn.shield_filter_node", "--ros-args",
            "-p", ["mode:=", mode],
            "-p", "obstacle_topic:=/shield/obstacles_state",
            "-p", ["robot_radius:=", robot_radius],
            "-p", ["u_max:=", u_max],
            "-p", ["beta:=", LaunchConfiguration("beta")],
            "-p", ["use_refined:=", LaunchConfiguration("use_refined")],
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="shield"),  # shield/shield_multi/nominal
        DeclareLaunchArgument("obstacle_ids", default_value="[2]"),
        DeclareLaunchArgument("obstacle_radii", default_value="[0.15]"),
        DeclareLaunchArgument("convert_y_up", default_value="true"),
        DeclareLaunchArgument("robot_radius", default_value="0.15"),
        DeclareLaunchArgument("u_max", default_value="0.5"),
        DeclareLaunchArgument("beta", default_value="1.0"),          # shield_multi 用
        DeclareLaunchArgument("use_refined", default_value="true"),  # shield_multi 用
        DeclareLaunchArgument("python_bin", default_value=default_python),

        obstacle_bridge,
        shield_filter,
    ])
