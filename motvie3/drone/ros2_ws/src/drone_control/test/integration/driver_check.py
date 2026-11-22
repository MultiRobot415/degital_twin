#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""tello_swarm_driver の結合テスト（偽 Tello 3 台が相手）。run_driver_check.sh から起動する。

    python3 driver_check.py <logdir> <fake3 の PID>
"""

import os
import signal
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction

logdir, fake3_pid = sys.argv[1], int(sys.argv[2])
NAMES = ["tello1", "tello2", "tello3"]
BAT = {"tello1": 81, "tello2": 82, "tello3": 83}

rclpy.init()
n = Node("driver_check")
bat, responses = {}, {k: [] for k in NAMES}
for k in NAMES:
    n.create_subscription(FlightData, f"/{k}/flight_data", lambda m, k=k: bat.__setitem__(k, m.bat), 10)
    n.create_subscription(TelloResponse, f"/{k}/tello_response",
                          lambda m, k=k: responses[k].append((m.rc, m.str)), 10)
cmd_pubs = {k: n.create_publisher(Twist, f"/{k}/cmd_vel", 10) for k in NAMES}
clients = {k: n.create_client(TelloAction, f"/{k}/tello_action") for k in NAMES}


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)


def fake_log(k):
    with open(os.path.join(logdir, f"{k}.log")) as f:
        return [line.split(" ", 1) for line in f.read().splitlines()]


def call(k, cmd):
    assert clients[k].wait_for_service(timeout_sec=5.0)
    f = clients[k].call_async(TelloAction.Request(cmd=cmd))
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    return f.result().rc


def twist(x=0.0, y=0.0, z=0.0, az=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z, t.angular.z = x, y, z, az
    return t


R = TelloAction.Response
ok = True
try:
    # 1) 接続: 全機に "command"、状態は送信元 IP で正しい機体に振り分けられる
    spin(4.0)
    assert bat == BAT, f"flight_data の振り分けが違う: {bat}"
    assert all(any(c == "command" for _, c in fake_log(k)) for k in NAMES)
    print("1) 接続・状態の IP 振り分け OK", bat)

    # 2) cmd_vel → rc の符号（左 +y → a 負, 反時計 +az → d 負）。他の機体には送られない
    cmd_pubs["tello2"].publish(twist(x=0.25, y=0.5, z=-0.1, az=0.3))
    spin(0.5)
    rc2 = [c for _, c in fake_log("tello2") if c.startswith("rc")]
    assert rc2 and rc2[-1] == "rc -50 25 -10 -30", rc2
    assert not [c for k in ("tello1", "tello3") for _, c in fake_log(k) if c.startswith("rc")]
    print("2) rc の符号・宛先 OK:", rc2[-1])

    # 3) 応答待ち中: rc を送らない / 新しいコマンドは BUSY / emergency だけは送る
    assert call("tello1", "takeoff") == R.OK
    t_takeoff = time.monotonic()
    for _ in range(10):
        cmd_pubs["tello1"].publish(twist(x=0.5))
        spin(0.03)
    assert call("tello1", "land") == R.ERROR_BUSY
    assert not [c for t, c in fake_log("tello1") if c.startswith("rc") and float(t) > t_takeoff - 0.5], \
        "応答待ち中に rc が送られている"
    assert call("tello1", "emergency") == R.OK
    spin(1.5)
    assert any(c == "emergency" for _, c in fake_log("tello1"))
    assert responses["tello1"] and responses["tello1"][0][0] == TelloResponse.OK, responses["tello1"]
    print("3) 応答待ち中の rc 抑止・BUSY・emergency の割り込み OK")

    # 4) キープアライブ: 12 秒何も送らなければ "rc 0 0 0 0"
    t_mark = time.monotonic()
    spin(14.0)
    for k in NAMES:
        assert any(c == "rc 0 0 0 0" and float(t) > t_mark for t, c in fake_log(k)), k
    print("4) キープアライブ（rc 0 0 0 0）OK")

    # 5) 状態が途絶えたら未接続 → NOT_CONNECTED、"command" を送り直し、復帰したら使える
    os.kill(fake3_pid, signal.SIGUSR1)       # fake3 の状態送信を停止
    t_mark = time.monotonic()
    spin(6.5)
    assert call("tello3", "takeoff") == R.ERROR_NOT_CONNECTED
    assert any(c == "command" and float(t) > t_mark for t, c in fake_log("tello3"))
    os.kill(fake3_pid, signal.SIGUSR1)       # 再開
    spin(2.5)
    assert call("tello3", "land") == R.OK
    print("5) 未接続の検出・再接続 OK")

    # 6) swarm.yaml に無い IP からのパケットは無視（ドライバは生きている）
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.9", 0))
    s.sendto(b"bat:1;", ("127.0.0.1", 8890))
    s.close()
    spin(0.5)
    assert bat["tello1"] == 81
    spin(1.5)
    assert call("tello2", "command") == R.OK
    print("6) 未知 IP の無視 OK")
except AssertionError as e:
    ok = False
    print("FAIL:", e)
finally:
    n.destroy_node()
    rclpy.shutdown()
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
