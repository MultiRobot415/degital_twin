# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""キーボードで /cmd_vel を出す自作テレオペノード（手動操作用）。

position_controller の代わりに「手で飛ばしたい」ときに使う。
出力は実機 tello_ros と同じ /cmd_vel (Twist, 正規化 -1〜1)。

  W / S : 前進 / 後退        A / D : 左 / 右
  R / F : 上昇 / 下降        Q / E : 左旋回 / 右旋回
  Space : 全停止             Ctrl-C: 終了

※ キー入力を端末から読むので、このノードは「自分の端末」で起動すること。
   （ros2 launch には含めない。launch プロセスにはキーボード端末が無いため）
※ position_controller と同時に動かさないこと（どちらも /cmd_vel に書いて競合する）。

実行:
    source /opt/ros/humble/setup.bash
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 run drone_control keyboard_teleop
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# キー → [vx, vy, vz, wz]（正規化）の増分マップ
KEY_BINDINGS = {
    "w": (1.0, 0.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0, 0.0),
    "d": (0.0, -1.0, 0.0, 0.0),
    "r": (0.0, 0.0, 1.0, 0.0),
    "f": (0.0, 0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 0.0, 1.0),
    "e": (0.0, 0.0, 0.0, -1.0),
}

HELP = __doc__


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("keyboard teleop: WSAD=平面 / RF=上下 / QE=旋回 / Space=停止")

    def publish(self, vx, vy, vz, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
        msg.angular.z = wz
        self.pub.publish(msg)


def get_key(timeout: float) -> str:
    """端末から1キーを非ブロッキングで読む（無ければ空文字）。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if rlist else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(timeout=0.1).lower()
            if key == "\x03":  # Ctrl-C
                break
            if key in KEY_BINDINGS:
                node.publish(*KEY_BINDINGS[key])
            elif key == " " or key == "":
                node.publish(0.0, 0.0, 0.0, 0.0)  # 停止（キーを離した＝入力なしも停止）
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0, 0.0, 0.0)  # 終了時に確実に止める
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
