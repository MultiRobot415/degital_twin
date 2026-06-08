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
        ],
    },
)
