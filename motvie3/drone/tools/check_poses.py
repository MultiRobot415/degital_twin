#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""飛行前チェック: 全機の位置・向き（yaw）・データの届き具合を 1 秒ごとに表示する（Ctrl-C で終了）。

    source ~/Downloads/degital_twin-main/drone/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=42
    python3 ~/Downloads/degital_twin-main/drone/tools/check_poses.py

座標は ROS 規約（x=前, y=左, z=上）。yaw は +x 向きで 0°、左回り（上から見て反時計回り）が正。
機首を +x に向けて置いたのに yaw が 0° 付近でなければ、Motive の剛体の向きがずれている（飛ばさないこと）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from drone_control import swarm_config
from drone_control.frames import yaw_from_quat


def main():
    swarm = swarm_config.load_swarm(swarm_config.default_swarm_file())
    rclpy.init()
    node = Node("check_poses")
    last, count = {}, {n: 0 for n in swarm.names}

    def cb(msg, name):
        last[name] = (msg, time.monotonic())
        count[name] += 1

    for n in swarm.names:
        node.create_subscription(PoseStamped, f"/{n}/pose", lambda m, n=n: cb(m, n), 10)
    print("位置 [m]（x 前 / y 左 / z 上）と yaw [deg]。機首を +x に向けて置いたら yaw ≈ 0° のはず。Ctrl-C で終了")
    try:
        t_next = time.monotonic() + 1.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() < t_next:
                continue
            t_next += 1.0
            rows = []
            for n in swarm.names:
                if n not in last:
                    rows.append(f"{n}: データなし（mocap / pose_bridge / 剛体 ID を確認）")
                    continue
                m, t = last[n]
                p, q = m.pose.position, m.pose.orientation
                yaw = math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))
                age = time.monotonic() - t
                stale = "  ← 止まっている（剛体が見えていない？）" if age > 0.5 else ""
                rows.append(f"{n}: x={p.x:+.2f} y={p.y:+.2f} z={p.z:+.2f}  yaw={yaw:+6.1f}°  "
                            f"{count[n]:4d} Hz{stale}")
                count[n] = 0
            print(" | ".join(rows) if len(rows) == 1 else "\n".join(rows) + "\n" + "-" * 60)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
