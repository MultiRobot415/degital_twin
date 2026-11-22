import os
from glob import glob

from setuptools import setup

package_name = "drone_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch ファイルと rviz 設定を install して ros2 launch から見えるようにする
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        # 群制御の設定（機体・隊形）。ノード/launch の既定値はこの install 先を読む
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "formation"), glob("config/formation/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="taisei",
    maintainer_email="taisei040428@gmail.com",
    description="位置制御・テレオペ（mocap pose <-> /cmd_vel）。Tello デジタルツイン用・実機/Sim共通",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # `ros2 run drone_control <名前>` で起動できる
            "position_controller = drone_control.position_controller:main",
            "keyboard_teleop = drone_control.keyboard_teleop:main",
            "pose_bridge = drone_control.pose_bridge:main",
            "natnet_receiver = drone_control.natnet_receiver:main",   # Motive → /rigid_bodies（受信だけ）
            # 群フォーメーション（Sim / 実機 共通の制御 + 各インフラ）
            "formation_agent = drone_control.formation_agent:main",
            "consensus_agent = drone_control.consensus_agent:main",   # 論文 第3章の合意フォーメーション
            "show_agent = drone_control.show_agent:main",             # ルールベースのドローンショー
            "formation_commander = drone_control.formation_commander:main",
            "swarm_sim = drone_control.swarm_sim:main",
            "tello_swarm_driver = drone_control.tello_swarm_driver:main",
            "plot_flight = drone_control.plot_flight:main",   # 記録した rosbag をグラフにする
        ],
    },
)
