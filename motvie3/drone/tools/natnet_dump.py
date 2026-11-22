#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Motive の NatNet ストリームを ROS / NatNet SDK を通さずに直接受信して中身を表示する（Ctrl-C で終了）。

    python3 ~/Downloads/degital_twin-main/drone/tools/natnet_dump.py            # 普通の確認
    python3 ~/Downloads/degital_twin-main/drone/tools/natnet_dump.py --hex      # 先頭バイトの 16 進ダンプも出す
    python3 ~/Downloads/degital_twin-main/drone/tools/natnet_dump.py --natnet 2.10   # Motive 1.x のとき

やること:
  ① Motive のコマンドポート(1510)に NAT_CONNECT を送り、返事（NAT_SERVERINFO）があればバージョンを表示する。
     返事が無くても受信はできる（2026-10-05 時点の Motive 3.x PC は返事が無い）
  ② 返事があり NatNet 4.1 以上なら、剛体の定義（名前 ↔ Streaming ID）も問い合わせる
  ③ マルチキャスト(239.255.42.99:1511)に参加し、フレームを解析して 1 秒ごとに表示する

解析は drone_control/natnet_protocol.py（natnet_receiver ノードと同じもの）。ROS の source は要らない。
表示する値は Motive の生の値（座標変換前。Up Axis = Y なら x=前, y=上, z=右）。
クォータニオンは NatNet の並び (qx, qy, qz, qw)。並びの違いは drone/docs/motive3_natnet.md。

mocap 受信ノード（swarm_real.launch.py）と同時に動かしてよい（マルチキャストは両方に届く）。
"""

import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "drone_control"))
from drone_control import natnet_protocol as nn  # noqa: E402


def yaw_about_up(q):
    """Motive(Y-up) のクォータニオン → 上(+Y)まわりの角度 [deg]（参考値。上から見て反時計回りが正）"""
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x)))


def hexdump(buf: bytes, n: int) -> str:
    return "\n".join(f"  {i:04x}  " + " ".join(f"{b:02x}" for b in buf[i:i + 16])
                     for i in range(0, min(n, len(buf)), 16))


def print_frame(f, ver, names, hz, raw, hex_bytes):
    print(f"frame {f.frame_number}  受信 {hz:.0f} Hz  NatNet {ver[0]}.{ver[1]}  "
          f"{len(raw) - 4} バイト中 {f.parsed_bytes} バイトまで解析")
    print(f"  marker sets: {list(f.marker_sets) or 'なし'}   unlabeled markers: {f.n_unlabeled_markers}")
    if not f.rigid_bodies:
        print("  剛体: 0 個（Motive の Streaming で Rigid Bodies が ON か、剛体が登録されているか）")
    for b in f.rigid_bodies:
        p, q = b.position, b.orientation
        qn = math.sqrt(sum(v * v for v in q))
        valid = {None: "-", True: "OK", False: "見失い"}[b.tracking_valid]
        name = f" '{names[b.id]}'" if b.id in names else ""
        bad = "  ← |q|≠1 解析ずれ?" if abs(qn - 1.0) > 1e-3 else ""
        print(f"  ID {b.id:3d}{name}: pos=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})  "
              f"q(x,y,z,w)=({q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f}) |q|={qn:.4f}  "
              f"yaw(+Y軸)={yaw_about_up(q):+6.1f}°  err={b.mean_error * 1000:.2f}mm  tracking={valid}{bad}")
    if hex_bytes:
        print(hexdump(raw, hex_bytes))
    print("-" * 80)


def query_server(args):
    """コマンドポートに問い合わせる。戻り値: (NatNet バージョン or None, {ID: 名前})"""
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.bind((args.local, 0))
    cmd.settimeout(1.0)
    ver, names = None, {}
    try:
        cmd.sendto(nn.pack_message(nn.NAT_CONNECT, b"Ping\0"), (args.server, args.command_port))
        msg_id, payload = nn.parse_message(cmd.recvfrom(65535)[0])
        if msg_id != nn.NAT_SERVERINFO:
            print(f"[サーバー] 想定外の応答 message id={msg_id}")
            return None, names
        app, app_ver, nn_ver = nn.parse_serverinfo(payload)
        print(f"[サーバー] {app}  ver {'.'.join(map(str, app_ver))}  NatNet {'.'.join(map(str, nn_ver))}")
        ver = (nn_ver[0], nn_ver[1])
        if ver >= (4, 1):
            cmd.sendto(nn.pack_message(nn.NAT_REQUEST_MODELDEF), (args.server, args.command_port))
            while True:
                msg_id, payload = nn.parse_message(cmd.recvfrom(65535)[0])
                if msg_id == nn.NAT_MODELDEF:
                    break
            rbs = nn.parse_modeldef_rigid_bodies(payload, ver) or []
            names = {i: n for i, n, _ in rbs}
            print("[剛体の定義] " + (", ".join(f"ID {i} = '{n}'" for i, n, _ in rbs) or "なし"))
    except socket.timeout:
        print(f"[サーバー] {args.server}:{args.command_port} から応答なし（受信には不要。バージョンは --natnet の値で解析）")
    except nn.NatNetError as e:
        print(f"[サーバー] 応答を解析できず: {e}")
    finally:
        cmd.close()
    return ver, names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default="192.168.11.101", help="Motive PC の IP")
    ap.add_argument("--local", default="192.168.11.19", help="この PC の IP（マルチキャストを受ける NIC）")
    ap.add_argument("--multicast", default="239.255.42.99")
    ap.add_argument("--command-port", type=int, default=1510)
    ap.add_argument("--data-port", type=int, default=1511)
    ap.add_argument("--natnet", default="4.1",
                    help="NatNet バージョン（Motive 3.x = 4.1, 1.x = 2.10）。Motive が返事をくれたらそちらを使う")
    ap.add_argument("--hex", type=int, nargs="?", const=128, default=0,
                    help="パケット先頭の 16 進ダンプを出す（バイト数。既定 128）")
    ap.add_argument("--period", type=float, default=1.0, help="表示間隔 [s]")
    args = ap.parse_args()

    server_ver, names = query_server(args)
    ver = server_ver or nn.parse_version(args.natnet)
    print(f"[設定] NatNet {ver[0]}.{ver[1]} として解析する")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        sock.bind(("", args.data_port))
    except OSError as e:
        raise SystemExit(f"ポート {args.data_port} を bind できない（{e}）。mocap ドライバを止めてから再実行") from None
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(args.multicast) + socket.inet_aton(args.local))
    sock.settimeout(2.0)
    print(f"[受信] {args.multicast}:{args.data_port} を {args.local} で待っている... Ctrl-C で終了\n")

    count, t0 = 0, time.monotonic()
    t_start = t0
    try:
        while True:
            try:
                data, src = sock.recvfrom(65535)
            except socket.timeout:
                print("[受信] 2 秒間パケットなし（Streaming OFF / Multicast でない / Local Interface 違い）")
                continue
            msg_id, payload = nn.parse_message(data)
            if msg_id != nn.NAT_FRAMEOFDATA:
                print(f"[受信] message id={msg_id} size={len(payload)} from {src[0]}（フレーム以外）")
                continue
            count += 1
            now = time.monotonic()
            if now - t0 < args.period:
                continue
            hz = count / (now - t0)
            count, t0 = 0, now
            try:
                print_frame(nn.parse_frame(payload, ver), ver, names, hz, data, args.hex)
            except nn.NatNetError as e:
                print(f"[解析失敗] {e}")
                print(hexdump(data, max(args.hex, 128)))
    except KeyboardInterrupt:
        print(f"\n終了（{time.monotonic() - t_start:.0f} 秒）")


if __name__ == "__main__":
    main()
