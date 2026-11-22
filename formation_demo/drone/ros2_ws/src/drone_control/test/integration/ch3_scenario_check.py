# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""論文 第3章の合意フォーメーションの Sim 結合テスト本体（run_ch3_scenario.sh から起動する）。

swarm_sim + consensus_agent×3 を相手に、commander の代わりに仮想 Leader（/formation/leader）を出し、
    1) 静止 Leader → 全機が r_L + d_i に集まる
    2) 楕円軌道（隊形が進行方向に合わせて回る）→ 追従誤差が小さい
    3) disable → その場で止まる / 着陸
を検証する。

    python3 ch3_scenario_check.py <formation>
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction

from drone_control import swarm_config
from drone_control.consensus_law import desired_offset
from drone_control.formation_agent import ENABLE_TOPIC, LATCHED_QOS
from drone_control.frames import quat_from_yaw
from drone_control.leader_trajectory import Ellipse, LeaderState

name = sys.argv[1]
swarm = swarm_config.load_swarm(swarm_config.default_swarm_file())
spec = swarm_config.load_consensus(swarm_config.formation_file(name), swarm)

rclpy.init()
n = Node("ch3_scenario_check")
pos = {}
for k in swarm.names:
    n.create_subscription(
        PoseStamped, f"/{k}/pose",
        lambda m, k=k: pos.__setitem__(k, (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 10)
leader_pub = n.create_publisher(Odometry, "/formation/leader", 10)
en_pub = n.create_publisher(Bool, ENABLE_TOPIC, LATCHED_QOS)
clients = {k: n.create_client(TelloAction, f"/{k}/tello_action") for k in swarm.names}
leader = LeaderState(tuple(swarm.reference_initial), (0.0, 0.0, 0.0), 0.0, 0.0)


def publish_leader(st: LeaderState):
    m = Odometry()
    m.header.stamp = n.get_clock().now().to_msg()
    m.header.frame_id = "world"
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = st.position
    q = quat_from_yaw(st.heading)
    o = m.pose.pose.orientation
    o.x, o.y, o.z, o.w = q
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = st.velocity
    m.twist.twist.angular.z = st.heading_rate
    leader_pub.publish(m)


def spin(sec, traj=None, t0=None, record=None):
    """sec 秒回す。traj があれば Leader を 20Hz で動かし、record に (誤差) を貯める。"""
    global leader
    end, nxt = time.time() + sec, 0.0
    while time.time() < end:
        now = time.time()
        if now >= nxt:
            nxt = now + 0.05
            if traj is not None:
                leader = traj.state(now - t0, leader.heading, swarm.heading_speed_threshold)
            publish_leader(leader)
            if record is not None and len(pos) == len(swarm.names):
                record.append(errors())
        rclpy.spin_once(n, timeout_sec=0.005)


def errors():
    out = {}
    for k, a in spec.agents.items():
        d = desired_offset(a, leader.heading, spec.rotate_with_leader)
        goal = [leader.position[i] + d[i] for i in range(3)]
        out[k] = math.sqrt(sum((pos[k][i] - goal[i]) ** 2 for i in range(3)))
    return out


def action(cmd):
    for k, c in clients.items():
        assert c.wait_for_service(timeout_sec=5.0), k
        f = c.call_async(TelloAction.Request(cmd=cmd))
        rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
        assert f.result().rc == TelloAction.Response.OK, (k, cmd)


ok = True
try:
    en_pub.publish(Bool(data=False))
    spin(2.0)
    action("takeoff")
    spin(3.0)
    en_pub.publish(Bool(data=True))
    spin(10.0)
    e = errors()
    print(f"[{name}] 1) 静止 Leader: 目標からの距離 " + ", ".join(f"{k}={v:.4f}m" for k, v in e.items()))
    assert max(e.values()) < 0.02, e

    ell = Ellipse(leader.position, leader.heading, swarm.ellipse_a, swarm.ellipse_b, swarm.ellipse_period)
    rec = []
    t0 = time.time()
    spin(25.0, traj=ell, t0=t0, record=rec)
    tail = rec[len(rec) // 2:]      # 後半（過渡を除く）
    worst = {k: max(r[k] for r in tail) for k in swarm.names}
    print(f"  2) 楕円 25s（進行方向 {math.degrees(leader.heading):.0f}° まで回転）: 後半の最大追従誤差 "
          + ", ".join(f"{k}={v:.4f}m" for k, v in worst.items()))
    assert max(worst.values()) < 0.03, worst

    leader = LeaderState(leader.position, (0.0, 0.0, 0.0), leader.heading, 0.0)
    en_pub.publish(Bool(data=False))
    spin(0.5)
    p_a = dict(pos)
    spin(2.0)
    drift = max(abs(pos[k][i] - p_a[k][i]) for k in swarm.names for i in range(3))
    print(f"  3) disable → 停止 drift={drift:.2e} m")
    assert drift < 1e-3, drift
    action("land")
    spin(max(p[2] for p in pos.values()) / 0.4 + 1.5)
    assert all(abs(p[2]) < 1e-6 for p in pos.values()), pos
    print("  4) 着陸 OK")
except AssertionError as ex:
    ok = False
    print("FAIL:", ex)
finally:
    n.destroy_node()
    rclpy.shutdown()
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
