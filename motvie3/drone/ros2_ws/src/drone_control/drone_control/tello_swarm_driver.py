# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""複数の Tello EDU（station モード）を 1 ノード・1 ソケットで扱う実機ドライバ。

なぜ tello_ros の tello_driver を機体数だけ起動しないのか:
    Tello は状態（state）を「command を送ってきた PC の UDP 8890 番」へ送る仕様で、ポートは変えられない。
    tello_driver は 1 プロセスが 8890 を bind するので、同じ PC で複数起動すると奪い合いになる。
    そこで ref/mocap_stack の TelloManager と同じく、コマンド用・状態用のソケットを 1 つずつ持ち、
    受信パケットは送信元 IP で機体に振り分ける。

ROS 側の窓口は tello_driver と同じ（名前空間だけ付く）ので、keyboard_teleop 等はリマップで使える:
    購読   /<name>/cmd_vel        (Twist, 正規化 -1〜1, 機体座標 ROS 規約) → "rc a b c d"
    提供   /<name>/tello_action   (tello_msgs/TelloAction: 任意の SDK コマンド。takeoff / land 等)
    配信   /<name>/tello_response (tello_msgs/TelloResponse: tello_action の最終応答)
           /<name>/flight_data    (tello_msgs/FlightData: 8890 の状態文字列をパース)

座標・符号: cmd_vel → rc の変換は frames.twist_to_tello_rc（左右とヨーの符号が反転する）。

振る舞いは tello_driver に合わせてある（機体ごとに独立）:
    - 状態が届いていない間は 1 秒ごとに "command" を送る（SDK モードに入れる。これで状態送信も始まる）
    - rc 以外のコマンドは応答待ち（waiting）になり、その間 rc と新しい tello_action は送らない（BUSY）
    - 応答が 9 秒来なければタイムアウト、状態が 4 秒来なければ未接続扱い
    - 12 秒何も送っていなければ "rc 0 0 0 0" を送る（Tello は 15 秒無通信で自動着陸するため）
    tello_driver と違う点は 1 つだけ:
    - "emergency" は応答待ち・未接続でも必ず送る（離陸応答待ちの最大 9 秒間に止められないのを避ける）
    ビデオは扱わない。

注意（Tello SDK の仕様による曖昧さ）:
    応答にはコマンドの識別子が無い。応答待ち中に emergency を割り込ませると、emergency への "ok" が
    先に待っていたコマンド（例: takeoff）の応答として扱われ、本来の応答は後で "Unexpected" と出る。
    emergency 後はモーターが止まっているので実害は無いが、ログを読むときは注意。

実行（通常は swarm_real.launch.py から起動する）:
    ros2 run drone_control tello_swarm_driver --ros-args -p swarm_file:=<path>
"""

import re
import signal
import socket
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction

from drone_control import swarm_config
from drone_control.frames import format_rc, twist_to_tello_rc

TELLO_COMMAND_PORT = 8889   # Tello 側のコマンド受付ポート（固定）
STATE_TIMEOUT = 4.0         # tello_driver と同じ値 [s]
KEEP_ALIVE = 12.0
COMMAND_TIMEOUT = 9.0
_FIELD_RE = re.compile(r"([^:;]+):([^;]+);")


@dataclass
class DroneLink:
    name: str
    ip: str
    waiting: bool = False          # rc 以外のコマンドの応答待ち
    respond: bool = False          # 応答を tello_response に流すか（tello_action 由来なら True）
    pending: str = ""
    send_time: float = 0.0         # 最後に何かを送った時刻（monotonic）
    state_time: Optional[float] = None
    connected: bool = False        # 状態が STATE_TIMEOUT 以内に届いている
    sdk: int = FlightData.SDK_UNKNOWN


class TelloSwarmDriver(Node):
    def __init__(self):
        super().__init__("tello_swarm_driver")
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("local_command_port", 8889)  # この PC 側のポート（Tello は送信元へ返信する）
        self.declare_parameter("state_port", 8890)          # Tello が状態を送ってくるポート（Tello 側で固定）
        self.declare_parameter("tello_command_port", TELLO_COMMAND_PORT)

        self.swarm = swarm_config.load_swarm(self.get_parameter("swarm_file").value)
        bind_address = self.get_parameter("bind_address").value
        local_port = int(self.get_parameter("local_command_port").value)
        state_port = int(self.get_parameter("state_port").value)
        self.tello_port = int(self.get_parameter("tello_command_port").value)

        self.links: Dict[str, DroneLink] = {d.name: DroneLink(d.name, d.ip) for d in self.swarm.drones}
        self.by_ip: Dict[str, DroneLink] = {link.ip: link for link in self.links.values()}
        self._unknown_ips = set()

        self.cmd_sock = self._udp_socket(bind_address, local_port)
        self.state_sock = self._udp_socket(bind_address, state_port)

        self.resp_pubs, self.fd_pubs = {}, {}
        for name in self.links:
            self.create_subscription(Twist, f"/{name}/cmd_vel", lambda m, n=name: self._on_cmd_vel(n, m), 10)
            self.create_service(TelloAction, f"/{name}/tello_action",
                                lambda req, res, n=name: self._on_action(n, req, res))
            self.resp_pubs[name] = self.create_publisher(TelloResponse, f"/{name}/tello_response", 10)
            self.fd_pubs[name] = self.create_publisher(FlightData, f"/{name}/flight_data", 10)

        self.create_timer(0.005, self._poll_sockets)   # 受信は非ブロッキングで吸い出す（単一スレッドで完結）
        self.create_timer(1.0, self._housekeeping)

        table = ", ".join(f"{link.name}={link.ip}" for link in self.links.values())
        self.get_logger().info(
            f"tello_swarm_driver: {table} | cmd {bind_address}:{local_port} → Tello:{self.tello_port}, "
            f"state {bind_address}:{state_port}")

    @staticmethod
    def _udp_socket(address: str, port: int) -> socket.socket:
        # SO_REUSEADDR は付けない: 付けると 2 つ目のドライバも同じポートを bind でき、
        # パケットの奪い合いが「エラーにならずに」起きてしまう。
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((address, port))
        except OSError as e:
            s.close()
            raise RuntimeError(
                f"UDP {address}:{port} を bind できない（{e}）。tello_driver / ref のスクリプト等が"
                f"同じポートを使っていないか確認") from e
        s.setblocking(False)
        return s

    # ------------------------------------------------------------------ 送信
    def _sendto(self, link: DroneLink, text: str) -> bool:
        try:
            self.cmd_sock.sendto(text.encode("utf-8"), (link.ip, self.tello_port))
        except OSError as e:
            self.get_logger().error(f"[{link.name}] '{text}' の送信に失敗: {e}", throttle_duration_sec=2.0)
            return False
        link.send_time = time.monotonic()
        return True

    def _initiate(self, link: DroneLink, text: str, respond: bool) -> None:
        """tello_driver の CommandSocket::initiate_command と同じ。応答待ち中は何もしない。"""
        if link.waiting:
            return
        if self._sendto(link, text) and not text.startswith("rc"):
            link.waiting, link.respond, link.pending = True, respond, text

    def _complete(self, link: DroneLink, rc: int, text: str) -> None:
        if link.respond:
            msg = TelloResponse()
            msg.rc, msg.str = rc, text
            self.resp_pubs[link.name].publish(msg)
        link.waiting, link.respond, link.pending = False, False, ""

    # ------------------------------------------------------------------ ROS 入力
    def _on_cmd_vel(self, name: str, msg: Twist):
        link = self.links[name]
        if link.waiting:
            return  # 離陸などの応答待ち中は rc を送らない（tello_driver と同じ）
        rc = twist_to_tello_rc(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        self._initiate(link, format_rc(rc), respond=False)

    def _on_action(self, name: str, req, res):
        link = self.links[name]
        cmd = req.cmd.strip()
        if cmd == "emergency":
            # 唯一の例外: 応答待ち・未接続でも送る
            if self._sendto(link, cmd) and not link.waiting:
                link.waiting, link.respond, link.pending = True, True, cmd
            self.get_logger().warn(f"[{name}] emergency を送信")
            res.rc = res.OK
        elif not link.connected:
            self.get_logger().warn(f"[{name}] Not connected, dropping '{cmd}'")
            res.rc = res.ERROR_NOT_CONNECTED
        elif link.waiting:
            self.get_logger().warn(f"[{name}] Busy（'{link.pending}' の応答待ち）, dropping '{cmd}'")
            res.rc = res.ERROR_BUSY
        else:
            self._initiate(link, cmd, respond=True)
            self.get_logger().info(f"[{name}] → '{cmd}'")
            res.rc = res.OK
        return res

    # ------------------------------------------------------------------ UDP 受信
    def _poll_sockets(self):
        for sock, handler in ((self.cmd_sock, self._on_command_packet), (self.state_sock, self._on_state_packet)):
            while True:
                try:
                    data, (ip, _port) = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                except OSError as e:
                    self.get_logger().error(f"受信エラー: {e}", throttle_duration_sec=2.0)
                    break
                link = self.by_ip.get(ip)
                if link is None:
                    if ip not in self._unknown_ips:
                        self._unknown_ips.add(ip)
                        self.get_logger().warn(f"swarm.yaml に無い IP {ip} からのパケットを無視する")
                    continue
                handler(link, data.decode("utf-8", errors="replace"))

    def _on_command_packet(self, link: DroneLink, text: str):
        text = text.strip()
        if link.waiting:
            self.get_logger().info(f"[{link.name}] '{link.pending}' ← '{text}'")
            self._complete(link, TelloResponse.ERROR if text == "error" else TelloResponse.OK, text)
        else:
            self.get_logger().warn(f"[{link.name}] Unexpected '{text}'", throttle_duration_sec=2.0)

    def _on_state_packet(self, link: DroneLink, raw: str):
        link.state_time = time.monotonic()
        fields = dict(_FIELD_RE.findall(raw))
        if not link.connected:
            link.connected = True
            link.sdk = (FlightData.SDK_2_0 if fields.get("mid", "257") != "257" else FlightData.SDK_1_3)
            self.get_logger().info(f"[{link.name}] 状態受信開始（接続）SDK={'2.0' if link.sdk == 2 else '1.3'}")

        msg = FlightData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.raw, msg.sdk = raw, link.sdk
        try:
            if link.sdk == FlightData.SDK_2_0:
                msg.mid, msg.x, msg.y, msg.z = (int(fields[k]) for k in ("mid", "x", "y", "z"))
            for k in ("pitch", "roll", "yaw", "vgx", "vgy", "vgz", "templ", "temph", "tof", "h", "bat", "time"):
                setattr(msg, k, int(fields[k]))
            for k in ("baro", "agx", "agy", "agz"):
                setattr(msg, k, float(fields[k]))
        except (KeyError, ValueError):
            self.get_logger().error(f"[{link.name}] 状態をパースできない: {raw!r}", throttle_duration_sec=5.0)
            return
        self.fd_pubs[link.name].publish(msg)

    # ------------------------------------------------------------------ 1Hz 管理
    def _housekeeping(self):
        now = time.monotonic()
        for link in self.links.values():
            if not link.connected and not link.waiting:
                self._initiate(link, "command", respond=False)  # 最初のコマンドは必ず "command"
                continue

            timed_out = False
            if link.waiting and now - link.send_time > COMMAND_TIMEOUT:
                self.get_logger().error(f"[{link.name}] '{link.pending}' がタイムアウト")
                self._complete(link, TelloResponse.TIMEOUT, "error: command timed out")
                timed_out = True
            if link.connected and link.state_time is not None and now - link.state_time > STATE_TIMEOUT:
                self.get_logger().error(f"[{link.name}] 状態が {STATE_TIMEOUT:.0f} 秒届かない（未接続扱い）")
                link.connected = False
                timed_out = True
            if timed_out:
                continue

            if link.connected and not link.waiting and now - link.send_time > KEEP_ALIVE:
                self._initiate(link, "rc 0 0 0 0", respond=False)

    def destroy_node(self):
        self.cmd_sock.close()
        self.state_sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TelloSwarmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C は端末と launch の両方から届くので、後片付けに入ったら 2 回目以降の割り込みは無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
