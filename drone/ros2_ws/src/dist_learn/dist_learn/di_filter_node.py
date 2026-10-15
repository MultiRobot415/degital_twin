# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""2重積分器（加速度入力）＋離散時間の高次CBF の安全フィルタ ROS2 ノード（第4回ゼミ用・単機）。

シミュレーション（~/202605_SHIELD/shield_reproduction/step5_double_integrator/）と**同じ本体**
`shield.di_filter.DIFilter` を import して使う。中身を変えるときは shield 側を直す。

    keyboard_teleop（/cmd_vel_nom にリマップ。T=離陸 L=着陸、R/F=高さ）
    RViz の 2D Goal Pose（/goal_pose）
        │
        ▼
    di_filter_node（20 Hz）
        ├ /tello/pose → 位置。速度は Kalman フィルタで推定（2重積分器のモデル）
        ├ ノミナル：目標へ向かう加速度  u_nom = (v_des - v) / tau_nom
        ├ DIFilter.step(x, v, u_nom, 障害物) → 安全な加速度 u
        ├ 加速度 → 速度指令：Tello は v' = (g c - v)/tau で指令 c に追従するので
        │      c = (v + tau u) / g        （機体座標の軸ごと。tau, g は 10/15 の実測）
        └ /cmd_vel（body 正規化）へ。高さ・旋回は teleop の値を素通し
        ▼ tello_driver → 実機

■ モデル規約
  - 位置 x=[px,py]（world, Z-up, m）。dt = 0.05（20 Hz。解析・シミュレーションと同じ）
  - tau_x=0.65, tau_y=0.74, g_x=1.48, g_y=1.52 は step0/stage2_entry.py（max_vxy=1.0 の規約）
    の値。機体座標の軸として使うので、ヨーは 0 付近に保つこと（teleop の Q/E で回さない）
  - 外乱の σ_d = 3.2 cm/s（1ステップ）, ±3σ で打ち切り（第2回の作り方）

■ 起動（scipy が要るので env_isaaclab の python で直起動。shield_filter_node と同じ）
    # 端末1：インフラ（飛ばない）
    ros2 launch drone_control bringup_real.launch.py rigid_body_id:=1 drone_ip:=192.168.11.50
    # 端末2：障害物（mocap の剛体を障害物にする場合）
    ros2 run drone_control obstacle_bridge --ros-args \\
        -p obstacle_ids:="[2]" -p obstacle_radii:="[0.15]" -p convert_y_up:=true
    # 端末3：このノード
    source /opt/ros/humble/setup.bash && source ~/research/drone/ros2_ws/install/setup.bash
    ~/env_isaaclab/bin/python -m dist_learn.di_filter_node --ros-args \\
        -p obstacle_topic:=/shield/obstacles_state
      （障害物を固定値で置くなら  -p obstacles:="[1.0, 0.0, 0.15]"）
    # 端末4：離着陸と高さ（生指令は /cmd_vel_nom へ。/cmd_vel を直接書かない）
    ros2 run drone_control keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_nom
  T で離陸 → R/F で高さを合わせる → RViz の 2D Goal Pose で目標を置くと動き出す。
  終了は L で着陸してから端末3を Ctrl-C（停止指令を出して記録を保存）。

■ 記録  log/di/<日付>/<時刻>.npz：時刻・位置（生）・速度推定・u_nom・u・指令・h・psi1・gamma2・
  解けたか・Freedman の状態など。外乱の解析（速度の式の残り）にもこのまま使える。
"""

from __future__ import annotations

import math
import os
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


def yaw_from_quat(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class AxisKF:
    """位置・速度の Kalman フィルタ（1軸）。x+ = x + dt v,  v+ = v + dt a + w。"""

    def __init__(self, dt, q_v, r_x):
        self.dt, self.q, self.r = dt, q_v ** 2, r_x ** 2
        self.s = None
        self.P = np.diag([1e-4, 1.0])

    def step(self, a, z):
        if self.s is None:
            self.s = np.array([z, 0.0])
            return self.s
        F = np.array([[1.0, self.dt], [0.0, 1.0]])
        self.s = F @ self.s + np.array([0.0, self.dt * a])
        self.P = F @ self.P @ F.T + np.diag([1e-8, self.q])
        S = self.P[0, 0] + self.r
        K = self.P[:, 0] / S
        self.s = self.s + K * (z - self.s[0])
        self.P = self.P - np.outer(K, self.P[0, :])
        return self.s


class DIFilterNode(Node):
    def __init__(self):
        super().__init__("di_filter_node")
        P = self.declare_parameter
        P("shield_path", os.path.expanduser("~/202605_SHIELD/shield_reproduction"))
        P("rate_hz", 20.0)
        P("max_vxy", 1.0)                 # /cmd_vel 正規化の規約（収集と同じ）
        P("tau_x", 0.65); P("tau_y", 0.74)
        P("g_x", 1.48); P("g_y", 1.52)
        P("v_cmd_max", 0.6)               # 速度指令の上限 [m/s, max_vxy 規約]（安全のため）
        P("obstacles", [1.0, 0.0, 0.15])  # [x,y,r,...]（obstacle_topic 指定時は使わない）
        P("obstacle_topic", "")
        P("robot_radius", 0.15)
        P("lam", 10.0); P("gam", 0.3); P("beta", 10.0)
        P("gamma1", 0.1); P("P", 0.01); P("K", 10)
        P("a_max", 2.3)
        P("sigma_d", 0.032)
        P("v_max", 0.5)                   # ノミナルの巡航速度 [m/s]
        P("tau_nom", 0.3)
        P("kf_r", 0.001)                  # mocap の位置ノイズ [m]
        P("pose_timeout", 0.3)
        P("record", True)
        P("log_root", os.path.expanduser("~/research/drone/ros2_ws/log"))

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        sp = g("shield_path")
        if sp not in sys.path:
            sys.path.insert(0, sp)
        from shield.di_filter import DIBarrier, DIFilter

        self.dt = 1.0 / float(g("rate_hz"))
        self.max_vxy = float(g("max_vxy"))
        self.tau = np.array([g("tau_x"), g("tau_y")], dtype=float)
        self.gain = np.array([g("g_x"), g("g_y")], dtype=float)
        self.v_cmd_max = float(g("v_cmd_max"))
        self.robot_r = float(g("robot_radius"))
        self.v_max = float(g("v_max"))
        self.tau_nom = float(g("tau_nom"))
        self.pose_timeout = float(g("pose_timeout"))
        self.filter = DIFilter(
            barrier=DIBarrier(lam=float(g("lam")), gam=float(g("gam")), beta=float(g("beta"))),
            dt=self.dt, gamma1=float(g("gamma1")), P=float(g("P")), K=int(g("K")),
            a_max=float(g("a_max")), sigma_d=float(g("sigma_d")))
        self.kf = [AxisKF(self.dt, float(g("sigma_d")), float(g("kf_r"))) for _ in range(2)]

        self.obstacle_topic = str(g("obstacle_topic"))
        self.obstacles = np.zeros((0, 3)) if self.obstacle_topic else \
            np.array(g("obstacles"), dtype=float).reshape(-1, 3)

        self.pos = None
        self.yaw = 0.0
        self.t_pose = -1.0
        self.goal = None
        self.u_prev = np.zeros(2)
        self.cmd_nom = np.zeros(4)

        self.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(Twist, "/cmd_vel_nom", self._on_nom, 10)
        if self.obstacle_topic:
            from std_msgs.msg import Float32MultiArray
            self.create_subscription(Float32MultiArray, self.obstacle_topic, self._on_obs, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(self.dt, self._tick)

        self.record = bool(g("record"))
        self.log_root = g("log_root")
        self._t0 = time.perf_counter()
        self._rec = {k: [] for k in ("t", "t_pose", "x_meas", "yaw", "x_est", "v_est",
                                     "u_nom", "u", "cmd", "h", "psi1", "gamma2", "feasible",
                                     "status", "bound", "sdf", "goal")}
        self.params = {n: g(n) for n in ("tau_x", "tau_y", "g_x", "g_y", "v_cmd_max",
                                         "robot_radius", "lam", "gam", "beta", "gamma1", "P",
                                         "K", "a_max", "sigma_d", "v_max", "tau_nom", "kf_r")}
        self.get_logger().info(
            f"di_filter_node up. dt={self.dt:.3f} gamma1={self.filter.gamma1} "
            f"P={self.filter.P} a_max={self.filter.a_max} obstacles={self.obstacles.tolist()} "
            f"| 2D Goal Pose を置くまでその場でホールド")

    # ---- callbacks ----
    def _on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.pos = np.array([p.x, p.y])
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.t_pose = time.perf_counter()

    def _on_goal(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.filter.reset()
        self.get_logger().info(f"goal = ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def _on_nom(self, msg):
        self.cmd_nom = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])

    def _on_obs(self, msg):
        if msg.data and len(msg.data) % 3 == 0:
            self.obstacles = np.array(msg.data, dtype=float).reshape(-1, 3)

    # ---- frames ----
    def _w2b(self, w):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c * w[0] + s * w[1], -s * w[0] + c * w[1]])

    def _tick(self):
        now = time.perf_counter()
        out = Twist()
        out.linear.z = float(self.cmd_nom[2])
        out.angular.z = float(self.cmd_nom[3])
        if self.pos is None or now - self.t_pose > self.pose_timeout:
            self.pub.publish(out)          # pose が無い・古い：水平は止める
            return

        # 速度の推定（前ステップに出した加速度でモデル予測 → mocap で補正）
        est = [kf.step(a, z) for kf, a, z in zip(self.kf, self.u_prev, self.pos)]
        x = np.array([est[0][0], est[1][0]])
        v = np.array([est[0][1], est[1][1]])
        if self.goal is None:
            self.goal = x.copy()           # 最初はその場でホールド

        v_des = self.goal - x
        nv = np.linalg.norm(v_des)
        v_des = v_des * min(1.0, self.v_max / max(nv, 1e-9)) * min(1.0, nv / 0.3)
        u_nom = np.clip((v_des - v) / self.tau_nom, -self.filter.a_max, self.filter.a_max)

        L = {"h": np.nan, "psi1": np.nan, "gamma2": np.nan, "feasible": True,
             "status": "no_obstacle", "bound": np.nan}
        if len(self.obstacles):
            rho, R = self.obstacles[:, :2], self.obstacles[:, 2] + self.robot_r
            u = self.filter.step(x, v, u_nom, rho, R)
            L = self.filter.last
            sdf = float(np.min(np.linalg.norm(self.pos - rho, axis=1) - R))
        else:
            u, sdf = u_nom, np.inf

        # 加速度 → 速度指令（機体座標の軸ごと）：c = (v + tau u) / g
        cb = (self._w2b(v) + self.tau * self._w2b(u)) / self.gain
        cb = np.clip(cb, -self.v_cmd_max, self.v_cmd_max)
        out.linear.x = float(np.clip(cb[0] / self.max_vxy, -1.0, 1.0))
        out.linear.y = float(np.clip(cb[1] / self.max_vxy, -1.0, 1.0))
        self.pub.publish(out)
        self.u_prev = u

        if L.get("status") not in (None, "fixed", "no_obstacle") and not L["feasible"]:
            self.get_logger().warn("制約を満たす加速度がない（最も安全側を出力）")
        if self.record:
            r = self._rec
            r["t"].append(now - self._t0); r["t_pose"].append(self.t_pose - self._t0)
            r["x_meas"].append(self.pos.copy()); r["yaw"].append(self.yaw)
            r["x_est"].append(x); r["v_est"].append(v)
            r["u_nom"].append(u_nom); r["u"].append(np.asarray(u, float))
            r["cmd"].append([out.linear.x, out.linear.y, out.linear.z, out.angular.z])
            for k in ("h", "psi1", "gamma2", "bound"):
                r[k].append(float(L[k]))
            r["feasible"].append(bool(L["feasible"])); r["status"].append(str(L["status"]))
            r["sdf"].append(sdf); r["goal"].append(self.goal.copy())

    def save(self):
        if not self.record or not self._rec["t"]:
            return
        now = datetime.now()
        d = os.path.join(self.log_root, "di", now.strftime("%Y-%m-%d"))
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, now.strftime("%H%M%S") + ".npz")
        r = self._rec
        np.savez(path, **{k: np.array(v) for k, v in r.items()},
                 obstacles=self.obstacles, dt=self.dt, params=str(self.params))
        n_bad = int(np.sum(np.array(r["sdf"]) < 0))
        self.get_logger().info(f"記録保存: {path}  (N={len(r['t'])}, sdf<0 が {n_bad} steps)")


def main():
    rclpy.init()
    node = DIFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        try:
            node.save()
        except Exception as e:
            print(f"[di_filter] 記録保存に失敗: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
