# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""NatNet 受信ノード: Motive のマルチキャストを直接受けて /rigid_bodies を出す（mocap4r2 ドライバの代わり）。

    Motive ──マルチキャスト 239.255.42.99:1511──▶ natnet_receiver ──/rigid_bodies──▶ pose_bridge × N

mocap4r2_optitrack_driver（NatNet SDK 3.0）は、接続時に Motive のコマンドポート(1510)から返事が来ないと
フレームを 1 つも出さない。Motive 3.x の PC はこの返事が届かない（2026-10-05 確認）ので、受信だけで済む
このノードに置き換えた。バージョンの問い合わせをしない代わりに natnet_version を設定で与える。

出力は mocap4r2 ドライバと同じ形（pose_bridge は無改造）:
    rigid_body_name = Streaming ID の文字列、pose = Motive の座標そのまま（変換は pose_bridge）

■ 見失った剛体（tracking valid = 0）は既定で出さない（drop_untracked:=true）。
  Motive が見失った剛体に最後の値や 0 を入れてくることがあるため。出さなければ pose_bridge も止まり、
  check_poses.py に「止まっている」と出る。

使い方:
    ros2 run drone_control natnet_receiver --ros-args -p natnet_version:=4.1
"""

import socket
import threading

import rclpy
from rclpy.node import Node

from mocap4r2_msgs.msg import RigidBodies, RigidBody

from drone_control import natnet_protocol as nn


class NatNetReceiver(Node):
    def __init__(self):
        super().__init__("natnet_receiver")

        self.declare_parameter("multicast_address", "239.255.42.99")
        self.declare_parameter("local_address", "192.168.11.19")   # マルチキャストを受ける NIC の IP
        self.declare_parameter("data_port", 1511)
        # Motive 3.x = "4.1"（実パケットで確認）, Motive 1.x = "2.10"。'2.10' が 2.1 にならないよう文字列で渡す
        self.declare_parameter("natnet_version", "4.1")
        self.declare_parameter("drop_untracked", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("output_topic", "/rigid_bodies")

        self.ver = nn.parse_version(self.get_parameter("natnet_version").value)
        self.drop_untracked = bool(self.get_parameter("drop_untracked").value)
        self.frame_id = self.get_parameter("frame_id").value
        mcast = self.get_parameter("multicast_address").value
        local = self.get_parameter("local_address").value
        port = int(self.get_parameter("data_port").value)

        self.pub = self.create_publisher(RigidBodies, self.get_parameter("output_topic").value, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(("", port))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(mcast) + socket.inet_aton(local))
        self.sock.settimeout(0.5)

        # 受信スレッドとウォッチドッグで共有する状態
        self._lock = threading.Lock()
        self._n_frames = 0
        self._n_errors = 0
        self._last_error = ""
        self._ids = set()
        self._lost = set()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f"natnet_receiver up. {mcast}:{port} を {local} で受信 -> {self.get_parameter('output_topic').value} | "
            f"NatNet {self.ver[0]}.{self.ver[1]} drop_untracked={self.drop_untracked}")

    def _recv_loop(self):
        while self._running:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break  # 終了時にソケットを閉じた
            try:
                msg_id, payload = nn.parse_message(packet)
                if msg_id != nn.NAT_FRAMEOFDATA:
                    continue
                frame = nn.parse_frame(payload, self.ver)
            except nn.NatNetError as e:
                with self._lock:
                    self._n_errors += 1
                    self._last_error = str(e)
                continue
            self._publish(frame)

    def _publish(self, frame: nn.Frame):
        msg = RigidBodies()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.frame_number = frame.frame_number & 0xFFFFFFFF
        lost = set()
        for b in frame.rigid_bodies:
            if b.tracking_valid is False:
                lost.add(b.id)
                if self.drop_untracked:
                    continue
            rb = RigidBody()
            rb.rigid_body_name = str(b.id)
            rb.pose.position.x, rb.pose.position.y, rb.pose.position.z = (float(v) for v in b.position)
            (rb.pose.orientation.x, rb.pose.orientation.y,
             rb.pose.orientation.z, rb.pose.orientation.w) = (float(v) for v in b.orientation)
            msg.rigidbodies.append(rb)
        self.pub.publish(msg)
        with self._lock:
            self._n_frames += 1
            self._ids = {b.id for b in frame.rigid_bodies}
            self._lost = lost

    def _watchdog(self):
        with self._lock:
            n, errs, last_err = self._n_frames, self._n_errors, self._last_error
            ids, lost = sorted(self._ids), sorted(self._lost)
            self._n_frames = self._n_errors = 0
        log = self.get_logger()
        if errs:
            log.error(f"2 秒で {errs} フレーム解析できなかった（natnet_version={self.ver[0]}.{self.ver[1]} が違う？）: "
                      f"{last_err}")
        if n == 0 and errs == 0:
            log.warn("2 秒間フレームが来ない（Motive の Streaming OFF / Multicast でない / local_address 違い）")
            return
        if lost:
            log.warn(f"Motive が見失っている剛体: {lost}"
                     + ("（/rigid_bodies に出していない）" if self.drop_untracked else ""))
        log.info(f"{n / 2.0:.0f} Hz, 剛体 ID {ids}", throttle_duration_sec=10.0)

    def destroy_node(self):
        self._running = False
        self.sock.close()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main():
    rclpy.init()
    node = NatNetReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
