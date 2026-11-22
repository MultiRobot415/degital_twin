#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""偽の Tello EDU（UDP の振る舞いだけを模擬）。tello_swarm_driver を機体なしで検証するため。

    python3 fake_tello.py --ip 127.0.0.2 --bat 81 --log /tmp/fake1.log

模擬する SDK の振る舞い:
    - :8889 でコマンドを受ける。rc 以外には応答（ok / 数値）を返す。rc には返さない
    - "command" を受けたら、送信元 IP の state_port(8890) へ 10Hz で状態文字列を送り始める
    - takeoff / land は --action-delay 秒後に "ok"（実機も数秒かかる）
    - SIGUSR1 で状態送信を止める / 再開する（ドライバの未接続判定の確認用）
受信したコマンドは 1 行ずつ --log に書く（"<monotonic時刻> <コマンド>"）。
"""

import argparse
import signal
import socket
import threading
import time

p = argparse.ArgumentParser()
p.add_argument("--ip", required=True)
p.add_argument("--port", type=int, default=8889)
p.add_argument("--state-port", type=int, default=8890)
p.add_argument("--bat", type=int, default=87)
p.add_argument("--action-delay", type=float, default=1.0)
p.add_argument("--log", required=True)
args = p.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.ip, args.port))
state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind((args.ip, 0))
log = open(args.log, "a", buffering=1)
state_target = None
state_paused = False
lock = threading.Lock()


def toggle_state(*_):
    global state_paused
    state_paused = not state_paused


signal.signal(signal.SIGUSR1, toggle_state)


def state_loop():
    while True:
        time.sleep(0.1)
        with lock:
            target = state_target
        if target is None or state_paused:
            continue
        s = (f"mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:1;roll:-2;yaw:3;vgx:0;vgy:0;vgz:0;templ:60;temph:62;"
             f"tof:10;h:0;bat:{args.bat};baro:1.23;time:0;agx:0.00;agy:0.00;agz:-1000.00;\r\n")
        state_sock.sendto(s.encode(), target)


def reply_later(text, addr, delay):
    time.sleep(delay)
    sock.sendto(text.encode(), addr)


threading.Thread(target=state_loop, daemon=True).start()
while True:
    data, addr = sock.recvfrom(1024)
    cmd = data.decode().strip()
    log.write(f"{time.monotonic():.3f} {cmd}\n")
    if cmd.startswith("rc "):
        continue
    if cmd == "command":
        with lock:
            state_target = (addr[0], args.state_port)
        sock.sendto(b"ok", addr)
    elif cmd in ("takeoff", "land"):
        threading.Thread(target=reply_later, args=("ok", addr, args.action_delay), daemon=True).start()
    elif cmd == "battery?":
        sock.sendto(str(args.bat).encode(), addr)
    else:
        sock.sendto(b"ok", addr)
