# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""キーボードで /cmd_vel を出す自作テレオペノード（加算式）＋離着陸。

操作モデル:
    キーを1回押すと、その方向に速度を「STEP だけ加算」する。
    もう一度押すとさらに加算（例: W を2回 → 前 +0.5）。逆キーで減算。
    Space で全方向ゼロ。離着陸は /tello_action サービスで送る（/cmd_vel とは別系統）。

実機 tello_ros との対応:
    /cmd_vel (Twist) は正規化 -1〜1（スティック倒し具合）。
      linear.x=前後, linear.y=左右, linear.z=上下, angular.z=旋回
    tello_sim.py は全倒し(1.0)=1 m/s（MAX_VXY=1.0）なので、
    STEP=0.25 は前後左右の「+0.25 m/s/回」に一致する。

  W / S : 前後 ±        A / D : 左右 ±
  R / F : 上下 ±        Q / E : 旋回 ±
  T : 離陸（takeoff）   L : 着陸（land）     Space : 全停止（全軸0）   Ctrl-C: 終了

  ※ 離着陸は実機（tello_driver が /tello_action を提供）でのみ有効。
    Sim では tello_action が無いので T/L は警告だけ出して何もしない。

※ 自分の端末で起動すること（ros2 launch には入れない）。
※ position_controller と同時に動かさないこと（/cmd_vel が競合する）。

実行:
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 run drone_control keyboard_teleop
    # 1回あたりの加算量を変える: --ros-args -p step:=0.2
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# tello_msgs は実機(tello_ros)をビルドした時だけ存在する。Sim単体では無いことがあるので、
# 無ければ離着陸機能だけ無効化して、テレオペ本体は動くようにする。
try:
    from tello_msgs.srv import TelloAction
    _HAS_TELLO_ACTION = True
except ImportError:
    TelloAction = None
    _HAS_TELLO_ACTION = False

# キー → (cmd配列のindex, 符号)。index: 0=前後, 1=左右, 2=上下, 3=旋回
KEY_DIR = {
    "w": (0, 1), "s": (0, -1),
    "a": (1, 1), "d": (1, -1),
    "r": (2, 1), "f": (2, -1),
    "q": (3, 1), "e": (3, -1),
}

HELP = __doc__
PUBLISH_PERIOD = 0.05  # 秒。保持している速度をこの間隔で流し続ける（20Hz）
ACTION_SERVICE = "/tello_action"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("step", 0.25)  # 1回押すごとの加算量（正規化）
        self.step = float(self.get_parameter("step").value)
        self.cmd = [0.0, 0.0, 0.0, 0.0]       # [前後, 左右, 上下, 旋回]（正規化 -1〜1）
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 離着陸サービスのクライアント（tello_msgs があるときだけ）
        self.action_client = None
        if _HAS_TELLO_ACTION:
            self.action_client = self.create_client(TelloAction, ACTION_SERVICE)

        self.get_logger().info(
            f"keyboard teleop (加算式) step={self.step} "
            f"離着陸={'有効' if self.action_client else '無効(tello_msgs無し)'}"
        )

    def add(self, idx, sign):
        self.cmd[idx] = clamp(self.cmd[idx] + sign * self.step, -1.0, 1.0)
        self.log_cmd()

    def stop(self):
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.log_cmd()

    def log_cmd(self):
        c = self.cmd
        self.get_logger().info(
            f"cmd: 前後={c[0]:+.2f} 左右={c[1]:+.2f} 上下={c[2]:+.2f} 旋回={c[3]:+.2f}"
        )

    def send_action(self, cmd: str):
        """/tello_action に takeoff/land 等を送る（非同期・/cmd_vel とは別系統）。"""
        if self.action_client is None:
            self.get_logger().warn(f"'{cmd}' は無効：tello_msgs が無い（Sim では離着陸不可）")
            return
        if not self.action_client.service_is_ready():
            self.get_logger().warn(
                f"'{cmd}' を送れない：{ACTION_SERVICE} が見つからない。tello_driver は起動してる？"
            )
            return
        req = TelloAction.Request()
        req.cmd = cmd
        self.action_client.call_async(req)  # 応答は待たない（ループを止めない）
        self.get_logger().info(f"→ {cmd} を送信")

    def publish(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.cmd[0], self.cmd[1], self.cmd[2]
        msg.angular.z = self.cmd[3]
        self.pub.publish(msg)


def get_key(timeout: float) -> str:
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
            key = get_key(timeout=PUBLISH_PERIOD).lower()

            if key == "\x03":          # Ctrl-C
                break
            elif key == " ":           # 全停止
                node.stop()
            elif key == "t":           # 離陸
                node.send_action("takeoff")
            elif key == "l":           # 着陸
                node.send_action("land")
            elif key in KEY_DIR:       # 該当軸に加算
                idx, sign = KEY_DIR[key]
                node.add(idx, sign)
            # キー無し("")は何もしない＝保持した速度を流し続ける

            node.publish()             # 毎ループ publish（保持＝動き続ける）
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.publish()                 # 終了時に確実に止める
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
