#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Tello EDU を station(子機)モードにする。研究室WiFiに参加させる一発スクリプト。

なぜ必要か:
    Tello は初期状態だと「自分でAPを出して待つ」モード（LED黄色点滅）。
    その状態だと、有線LAN上の制御PCからは見えない。
    SDKコマンド `ap <SSID> <PASS>` を送ると、Tello は研究室のWiFiルータに
    "子機" として参加し、ルータから 192.168.x.x のIPをもらう。
    こうすると有線の制御PCと同じLANに乗り、tello_driver から触れるようになる。

使い方（WiFi付きのノートPCで実行）:
    1. ノートPCのWiFiを Tello のAP "TELLO-XXXXXX" に繋ぐ（初期はパスワード無し）
    2. このスクリプトを実行:
         python3 tello_set_station.py "<研究室SSID>" "<パスワード>"
       ※ パスワードは引数で渡す（ファイルに書かない）。SSID/PASSにスペースや記号が
         あるときは必ずクォートで囲む。
    3. "ok" が2回返れば成功。Tello が自動で再起動して研究室WiFiに参加する。
    4. 制御PC側でルータのDHCP一覧 or スキャンで Tello の新しいIPを確認し、
         ros2 run ... drone_ip:=<そのIP>  で driver を起動。

元に戻す（APモードに戻したい時）:
    Tello本体の電源ボタン長押し(約5秒)で工場リセット → 再びAPモードに戻る。
"""

import socket
import sys

TELLO_AP_IP = "192.168.10.1"   # APモード時の Tello 自身のIP（固定）
TELLO_CMD_PORT = 8889          # SDK コマンドポート


def send(sock: socket.socket, msg: str, timeout: float = 8.0) -> str:
    """コマンドを送って応答文字列を返す。タイムアウトで例外。"""
    sock.sendto(msg.encode("utf-8"), (TELLO_AP_IP, TELLO_CMD_PORT))
    sock.settimeout(timeout)
    data, _ = sock.recvfrom(1024)
    return data.decode("utf-8", errors="replace").strip()


def main() -> int:
    if len(sys.argv) != 3:
        print('使い方: python3 tello_set_station.py "<SSID>" "<パスワード>"')
        return 2
    ssid, password = sys.argv[1], sys.argv[2]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", TELLO_CMD_PORT))  # Tello は送信元ポート(8889)に返事する

    try:
        print("→ SDKモードに入る (command) ...")
        resp = send(sock, "command")
        if resp.lower() != "ok":
            print(f"  期待した 'ok' が来ない: '{resp}'。TelloのAPに繋がっているか確認")
            return 1
        print("  ok")

        # パスワードはログに出さない（伏せる）
        print(f"→ station モードに設定 (ap {ssid} ****) ...")
        resp = send(sock, f"ap {ssid} {password}")
        # 成功応答は "ok" だけでなく "OK,drone will reboot in 3s" のように返る機体もある。
        # 失敗は "error"/"false" 等なので、ok を含むかどうかで判定する。
        rl = resp.lower()
        if not (rl.startswith("ok") or "reboot" in rl):
            print(f"  失敗: '{resp}'。SSID/パスワードを確認")
            return 1
        print(f"  応答: '{resp}'")

        print("\n✅ 成功。Tello が再起動して研究室WiFiに参加する。")
        print("   数十秒待ってから、制御PCでルータのDHCP一覧 or スキャンで新IPを確認。")
        return 0

    except socket.timeout:
        print("✗ 応答タイムアウト。"
              "①ノートPCのWiFiが 'TELLO-XXXX' に繋がっているか "
              "②Tello の電源が入っているか を確認")
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
