# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""複数エージェント用 キーボード司令塔：1つのキーボードで全機に一斉指示する。

これは keyboard_teleop の“複数機・非cmd_vel”版。position_controller が各機の
/telloN/cmd_vel_nom を握っているので、このノードは **cmd_vel を一切出さない**
（出すと競合する）。代わりに離着陸サービスと goal_pose を全機へ同時ファンアウトする:

    T : 全機 takeoff       L : 全機 land
    C : 全機 交差ゴールへ（go cross。各機の cross_x/cross_y を /telloN/goal_pose へ）
    B : 全機 スタートへ戻す（back home。home_x/home_y を送る＝往復でログを何本も取れる）
    E : 全機 emergency（モータ即停止＝墜落。緊急時のみ）
    H : ヘルプ再表示       Ctrl-C : 終了

なぜ cmd_vel を出さないか:
    交差デモは自律（position_controller が目標へ P 制御）。手動速度は使わない。
    手動で1機だけ飛ばしたいときは従来の keyboard_teleop を別 namespace に remap して使う。

対応関係:
    takeoff/land/emergency → /telloN/tello_action (tello_msgs/srv/TelloAction)
    交差/帰還              → /telloN/goal_pose (geometry_msgs/PoseStamped, x,y のみ・高さは維持)

実行（multiagent.launch.py を上げた別ターミナルで）:
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 run drone_control multiagent_teleop   # 既定で3機ぶんの S/G が入っている
    # 機体やゴールを変える例（会場に合わせて。座標は ROS(x,y)）:
    ros2 run drone_control multiagent_teleop --ros-args \\
        -p namespaces:="[tello1, tello2, tello3]" \\
        -p cross_x:="[0.7, 0.7, -0.7]" -p cross_y:="[-0.7, 0.7, -0.7]" \\
        -p home_x:="[-0.7, -0.7, 0.7]" -p home_y:="[0.7, -0.7, 0.0]"

※ multiagent.launch.py の goalN_*（＝ゴールG）と teleop の cross_* を一致させておくと挙動が揃う。
  home_*（Bキー）は各機のスタートS。生mocap(z,x)→ROS(x,y)変換は ros_x=raw_x, ros_y=-raw_z。
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    from tello_msgs.srv import TelloAction
    _HAS_TELLO_ACTION = True
except ImportError:
    TelloAction = None
    _HAS_TELLO_ACTION = False

HELP = __doc__
LOOP_PERIOD = 0.05  # 秒。キー待ちのタイムアウト（20Hz でスピン）


class MultiAgentTeleop(Node):
    def __init__(self):
        super().__init__("multiagent_teleop")

        # 対象機体（namespace のリスト）と、交差(=ゴールG)/帰還(=スタートS)地点。
        # ★座標は ROS(x,y)。会場で使う生mocapは (z,x) 表記なので必ず変換する:
        #     ros_x = raw_x ,  ros_y = -raw_z
        #   S1 raw(-0.7,-0.7)→(-0.7, 0.7)  S2 raw( 0.7,-0.7)→(-0.7,-0.7)  S3 raw(0,0.7)→(0.7,0.0)
        #   G1 raw( 0.7, 0.7)→( 0.7,-0.7)  G2 raw(-0.7, 0.7)→( 0.7, 0.7)  G3 raw(0.7,-0.7)→(-0.7,-0.7)
        #   ※ G3 は S2 と同一点。tello2 が離脱した後に tello3 が入る。
        self.declare_parameter("namespaces", ["tello1", "tello2", "tello3"])
        self.declare_parameter("cross_x", [0.7, 0.7, -0.7])    # ゴールG (Cキー)
        self.declare_parameter("cross_y", [-0.7, 0.7, -0.7])
        self.declare_parameter("home_x", [-0.7, -0.7, 0.7])    # スタートS (Bキー)
        self.declare_parameter("home_y", [0.7, -0.7, 0.0])
        self.declare_parameter("world_frame", "world")

        self.namespaces = [str(n) for n in self.get_parameter("namespaces").value]
        self.cross_x = [float(v) for v in self.get_parameter("cross_x").value]
        self.cross_y = [float(v) for v in self.get_parameter("cross_y").value]
        self.home_x = [float(v) for v in self.get_parameter("home_x").value]
        self.home_y = [float(v) for v in self.get_parameter("home_y").value]
        self.world_frame = str(self.get_parameter("world_frame").value)

        n = len(self.namespaces)
        for name, arr in (("cross_x", self.cross_x), ("cross_y", self.cross_y),
                          ("home_x", self.home_x), ("home_y", self.home_y)):
            if len(arr) != n:
                raise ValueError(
                    f"{name} は namespaces と同じ長さで渡す: "
                    f"namespaces={self.namespaces} {name}={arr}"
                )

        # 機体ごとに goal 配信 publisher と 離着陸サービス client を用意
        self.goal_pubs = {
            ns: self.create_publisher(PoseStamped, f"/{ns}/goal_pose", 10)
            for ns in self.namespaces
        }
        self.action_clients = {}
        if _HAS_TELLO_ACTION:
            self.action_clients = {
                ns: self.create_client(TelloAction, f"/{ns}/tello_action")
                for ns in self.namespaces
            }

        self.get_logger().info(
            f"multiagent teleop up. 機体={self.namespaces} "
            f"離着陸={'有効' if self.action_clients else '無効(tello_msgs無し)'}"
        )

    # ---- 全機に離着陸コマンドを一斉送信 ----
    def send_action_all(self, cmd: str):
        if not self.action_clients:
            self.get_logger().warn(f"'{cmd}' 不可：tello_msgs が無い（実機 driver を上げてる？）")
            return
        for ns, cli in self.action_clients.items():
            if not cli.service_is_ready():
                self.get_logger().warn(f"[{ns}] {cmd} を送れない：/{ns}/tello_action 未検出")
                continue
            req = TelloAction.Request()
            req.cmd = cmd
            cli.call_async(req)   # 応答は待たない（ループを止めない＝ほぼ同時に発火）
        self.get_logger().info(f"→ 全機に '{cmd}' を送信")

    # ---- 全機に goal_pose を一斉送信（xs/ys は機体順のリスト）----
    def send_goals(self, xs, ys, label: str):
        now = self.get_clock().now().to_msg()
        for ns, x, y in zip(self.namespaces, xs, ys):
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            self.goal_pubs[ns].publish(ps)
        pairs = ", ".join(f"{ns}->({x:+.2f},{y:+.2f})"
                          for ns, x, y in zip(self.namespaces, xs, ys))
        self.get_logger().info(f"→ {label}: {pairs}")


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
    node = MultiAgentTeleop()
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(timeout=LOOP_PERIOD).lower()

            if key == "\x03":          # Ctrl-C
                break
            elif key == "t":           # 全機離陸
                node.send_action_all("takeoff")
            elif key == "l":           # 全機着陸
                node.send_action_all("land")
            elif key == "c":           # 全機 交差ゴールへ
                node.send_goals(node.cross_x, node.cross_y, "go cross")
            elif key == "b":           # 全機 スタートへ戻す
                node.send_goals(node.home_x, node.home_y, "back home")
            elif key == "e":           # 全機 emergency（モータ停止）
                node.send_action_all("emergency")
            elif key == "h":           # ヘルプ
                print(HELP)

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
