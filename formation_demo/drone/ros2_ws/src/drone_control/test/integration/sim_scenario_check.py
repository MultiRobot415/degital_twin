# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Sim の結合テスト本体（run_sim_scenario.sh から起動する）。

swarm_sim + formation_agent×3 を相手に、formation_commander と同じ操作（離陸・enable・基準移動・
disable・着陸）をスクリプトで行い、各段階で全機が r + d_i にいるかを検証する。

    python3 sim_scenario_check.py <formation> <収束待ち秒>
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction

from drone_control import swarm_config
from drone_control.formation_agent import ENABLE_TOPIC, LATCHED_QOS, REFERENCE_TOPIC

formation_name = sys.argv[1]
settle = float(sys.argv[2])
swarm = swarm_config.load_swarm(swarm_config.default_swarm_file())
spec = swarm_config.load_formation(swarm_config.formation_file(formation_name), swarm)

rclpy.init()
n = Node("scenario_check")
pos, cmd_count = {}, {k: 0 for k in swarm.names}
for name in swarm.names:
    n.create_subscription(
        PoseStamped, f"/{name}/pose",
        lambda m, k=name: pos.__setitem__(k, (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 10)
    n.create_subscription(Twist, f"/{name}/cmd_vel", lambda m, k=name: cmd_count.__setitem__(k, cmd_count[k] + 1), 10)
ref_pub = n.create_publisher(PoseStamped, REFERENCE_TOPIC, LATCHED_QOS)
en_pub = n.create_publisher(Bool, ENABLE_TOPIC, LATCHED_QOS)
clients = {k: n.create_client(TelloAction, f"/{k}/tello_action") for k in swarm.names}


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)


def set_ref(r):
    m = PoseStamped()
    m.header.frame_id = "world"
    m.pose.position.x, m.pose.position.y, m.pose.position.z = r
    m.pose.orientation.w = 1.0
    ref_pub.publish(m)


def action(cmd):
    for k, c in clients.items():
        assert c.wait_for_service(timeout_sec=5.0), k
        f = c.call_async(TelloAction.Request(cmd=cmd))
        rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
        assert f.result().rc == TelloAction.Response.OK, (k, cmd, f.result().rc)


def check(r, label, tol=0.02):
    worst = 0.0
    for k, a in spec.agents.items():
        goal = [r[i] + a.offset[i] for i in range(3)]
        err = max(abs(pos[k][i] - goal[i]) for i in range(3))
        worst = max(worst, err)
        print(f"  {label} {k}: pos=({pos[k][0]:+.3f},{pos[k][1]:+.3f},{pos[k][2]:+.3f}) "
              f"goal=({goal[0]:+.3f},{goal[1]:+.3f},{goal[2]:+.3f}) err={err:.4f}")
    assert worst < tol, f"{label}: worst err {worst:.4f} >= {tol}"


ok = True
try:
    en_pub.publish(Bool(data=False))
    r0 = swarm.reference_initial
    set_ref(r0)
    spin(2.0)
    assert set(pos) == set(swarm.names), f"pose が来ない: {pos}"
    before = dict(cmd_count)
    spin(1.0)
    assert cmd_count == before, "enable=false なのに cmd_vel が出ている"
    print(f"[{formation_name}] 1) enable=false の間は cmd_vel なし OK")

    action("takeoff")
    spin(3.0)
    assert all(abs(pos[k][2] - 0.8) < 1e-6 for k in swarm.names), pos
    print("  2) 離陸 → 全機 z=0.8 OK")

    en_pub.publish(Bool(data=True))
    spin(settle)
    check(r0, "3) 初期隊形")

    r1 = (r0[0] - 0.6, r0[1] + 0.5, r0[2] + 0.2)
    set_ref(r1)
    spin(settle)
    check(r1, "4) 基準移動後")

    en_pub.publish(Bool(data=False))
    spin(0.5)
    p_a = dict(pos)
    spin(2.0)
    drift = max(abs(pos[k][i] - p_a[k][i]) for k in swarm.names for i in range(3))
    # 1次遅れ(τ=0.15s)で残速度が減衰する分だけ動く（理論値 ~1e-6 m）。それ以上なら指令が残っている
    assert drift < 1e-4, f"disable 後も動いている drift={drift}"
    print(f"  5) disable → 0 指令でその場に停止 OK (drift={drift:.2e})")

    action("land")
    # 着陸時間 = 現在の高さ / land_speed(0.4 m/s)。境界ぎりぎりにならないよう 1.5 秒の余裕を足す
    spin(max(p[2] for p in pos.values()) / 0.4 + 1.5)
    assert all(abs(pos[k][2]) < 1e-6 for k in swarm.names), pos
    print("  6) 着陸 → z=0 OK")
except AssertionError as e:
    ok = False
    print("FAIL:", e)
finally:
    n.destroy_node()
    rclpy.shutdown()
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
