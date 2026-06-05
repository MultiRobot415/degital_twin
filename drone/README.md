# drone — Isaac Sim 側のドローン開発（colcon の外）

Isaac Sim / Isaac Lab を起動して動かすスクリプト群を置く場所。
ここは ROS2 パッケージ（colcon）**ではない**。venv の python で直接実行する。

## 構成
```
drone/
├── sim/        起動スクリプト（python で直接実行する standalone）
│   └── run_drone_teleop.py
├── configs/    機体・制御パラメータ（将来）
└── README.md
```

ROS2 ノード（送信側 teleop など自作ノード）は `~/research/ros2_ws/src/` に置く。

## 実行
```bash
source /home/utsubo/env_isaaclab/bin/activate
source /opt/ros/humble/setup.bash

# Step1: まずホバリングだけ確認（ROSなし）
python ~/research/drone/sim/run_drone_teleop.py --no_ros

# Step2: ROS2 テレオペ（別端末で teleop_twist_keyboard を起動）
python ~/research/drone/sim/run_drone_teleop.py
#   別端末:
#   source /opt/ros/humble/setup.bash
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
