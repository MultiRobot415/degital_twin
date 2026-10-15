# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""2重積分器として動かしたときの「指令した加速度」と「実際の動き」を測る計測スクリプト（単機）。

離着陸と高さは keyboard_teleop（/cmd_vel_nom にリマップ）で行い、このスクリプトの端末で
Enter を押すと、決まった順番の動き（前後・左右の加速と減速、ヨーの回転）を自動で流す。
その間の指令と、mocap の生の位置（約 120 Hz、全フレーム）を記録する。

    keyboard_teleop ──/cmd_vel_nom──▶ di_excite ──/cmd_vel──▶ tello_driver
       (T/L, R/F)                        ▲ 待機中は teleop をそのまま流す
                       /tello/pose ──────┘ 計測中は自動の指令（高さは teleop のまま）

■ 自動で流す動き（既定。--amps で加速度の大きさを変えられる）
  加速度 a = 0.5, 1.0, 1.3 m/s^2 それぞれについて、機体座標の +x, -x, +y, -y の順に
      その場で静止（1.5 s） → +a で加速（T 秒） → -a で減速（T 秒） → 0（0.5 s）
  T は移動量 a T^2 が --disp（既定 0.15 m）になるよう決める。最後にヨー角速度 ±0.5, ±1.0
  （正規化）を 2 秒ずつ（その間、位置はその場に保つ）。全体でおよそ 50 秒。
  エリア 1.4 m 四方の中央から始めること。Tello の遅れのモデル（tau, g は 10/15 の実測）で模擬すると、
  開始位置から最大 0.32 m（機体の半径 0.15 m を足しても壁まで 0.2 m 余る）。
  既定の組（a ≤ 1.3、速度指令の上限 --v-cmd-max 1.0 = スティック全開）では、模擬で指令が上限に
  当たるステップは 0。a = 1.5 で 1 割、2.0 ではほぼ全部が当たる（全開でも、止まった状態から約 2.3、
  0.5 m/s 出ていると約 1.5 m/s^2 しか出ない）。実機の g が 1.5 より小さいと当たりうるが、
  当たったかは記録の cmd が ±1.0 かで分かる。
  加速度は、Tello が速度指令 c に v' = (g c - v)/tau で追従し、指令は dt の間一定とみて
  c = v/g + dt u / ((1 - exp(-dt/tau)) g) に直して送る（dt が tau より十分小さければ
  c ≈ (v + (tau + dt/2) u)/g。--no-zoh で連続時間の式 c = (v + tau u)/g）。

■ 速度は Kalman ではなく、直近 0.05 s の mocap の生の位置（約6点）の直線の傾きで出す。
  モデルを使わないので、測りたい「指令と実際の差」に推定の側からモデルが混ざらない。

■ 安全
  - 計測中に teleop の前後・左右・旋回が 0 でなくなったら中断（その場で静止）。Space で 0 に戻せる
  - この端末で a + Enter でも中断。開始位置から --fence m 以上離れたら中断
  - pose が 0.2 s 以上来なければ水平の指令を 0 にして中断

■ 起動
    # 端末1：インフラ
    ros2 launch drone_control bringup_real.launch.py rigid_body_id:=1 drone_ip:=192.168.11.50
    # 端末2：このスクリプト（numpy だけで動く）
    ros2 run dist_learn di_excite
    # 端末3：離着陸と高さ
    ros2 run drone_control keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_nom
  T で離陸 → R/F で高さを合わせて Space → 端末2 で Enter → 終わったら L で着陸 → 端末2 で q + Enter。
  何度でも Enter で繰り返せる。記録は 1 回ごとに log/di_excite/<日付>/<時刻>.npz。

■ 記録（npz）
  pose_t, pose (N,4) = [x, y, z, yaw]   mocap の全フレーム（受信時刻）
  t, seg, kind, u_body, u_world, v_est, x, yaw, cmd   20 Hz の制御ごと
      kind: 0=静止（位置の保持・速度指令）, 1=加速度, 2=ヨー
      u_* は kind=1 のときだけ意味がある（指令した加速度）
  segments（文字列）, params（文字列）
  解析：pose から速度を作り、kind=1 の区間で  d_k = v_{k+1} - v_k - dt u_world_k  を見る。
  速度は 120 Hz の生の位置の隣どうしの差分で作らないこと。位置のノイズ 0.3 mm が ×120 されて
  1点あたり数 cm/s の揺れになり、d がそれで埋もれる（模擬で確認：差分 5 cm/s → 0.1 s の直線あてはめ
  1.5 cm/s）。各時刻の前後 0.05 s（計 0.1 s）の位置に直線をあてはめた傾きを使う。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


def yaw_from_quat(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def build_program(amps, disp=0.4, t_max=0.6, yaw_rates=(0.5, 1.0)):
    """[(kind, value(2,) or float, duration)]  kind: hold / acc / yaw."""
    prog = []
    dirs = [np.array([1.0, 0.0]), np.array([-1.0, 0.0]),
            np.array([0.0, 1.0]), np.array([0.0, -1.0])]
    for a in amps:
        T = min(t_max, math.sqrt(disp / a))
        for d in dirs:
            prog += [("hold", None, 1.5), ("acc", a * d, T), ("acc", -a * d, T),
                     ("acc", np.zeros(2), 0.5)]
    for w in yaw_rates:
        prog += [("hold", None, 1.5), ("yaw", w, 2.0), ("yaw", -w, 2.0)]
    prog += [("hold", None, 1.5)]
    return prog


class DIExcite(Node):
    def __init__(self, args):
        super().__init__("di_excite")
        self.a = args
        self.dt = 1.0 / args.rate
        self.tau = np.array([args.tau_x, args.tau_y])
        self.gain = np.array([args.g_x, args.g_y])
        self.program = build_program(args.amps, disp=args.disp)
        self.poses = []                 # (t, x, y, z, yaw) 全フレーム
        self.nom = np.zeros(4)
        self.running = False
        self.quit = False
        self.lock = threading.Lock()
        self.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 50)
        self.create_subscription(Twist, "/cmd_vel_nom", self._on_nom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(self.dt, self._tick)
        self._t0 = time.perf_counter()
        total = sum(p[2] for p in self.program)
        self.get_logger().info(
            f"di_excite up. {len(self.program)} 区間・約 {total:.0f} 秒。"
            f"離陸して高さを合わせ Space → この端末で Enter で開始（a+Enter 中断, q+Enter 終了）")

    # ---- callbacks ----
    def _on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.poses.append((time.perf_counter() - self._t0, p.x, p.y, p.z,
                               yaw_from_quat(q.x, q.y, q.z, q.w)))

    def _on_nom(self, msg):
        self.nom = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])

    # ---- helpers ----
    def _state(self):
        """最新の位置・ヨーと、直近 window 秒の生の位置の直線の傾き（速度, world）。"""
        with self.lock:
            if not self.poses:
                return None
            arr = np.array(self.poses[-40:])
        t_now = time.perf_counter() - self._t0
        if t_now - arr[-1, 0] > 0.2:
            return None
        sel = arr[arr[:, 0] >= arr[-1, 0] - self.a.window]
        if len(sel) >= 3:
            tt = sel[:, 0] - sel[:, 0].mean()
            v = np.array([np.polyfit(tt, sel[:, i], 1)[0] for i in (1, 2)])
        else:
            v = np.zeros(2)
        return arr[-1, 1:3], arr[-1, 4], v

    def _rot(self, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s], [s, c]])            # body -> world

    def start(self):
        st = self._state()
        if st is None:
            self.get_logger().warn("pose が来ていない。開始しない")
            return
        if np.any(np.abs(self.nom[[0, 1, 3]]) > 1e-6):
            self.get_logger().warn("teleop の前後・左右・旋回が 0 でない。Space を押してから")
            return
        self.anchor, self.yaw0 = st[0].copy(), st[1]
        self.seg, self.t_seg = 0, time.perf_counter()
        self.rec = {k: [] for k in ("t", "seg", "kind", "u_body", "u_world", "v_est",
                                    "x", "yaw", "cmd")}
        with self.lock:
            self.pose_mark = len(self.poses)
        self.running = True
        self.get_logger().info("計測開始")

    def stop(self, why):
        if not self.running:
            return
        self.running = False
        self.pub.publish(Twist())
        self.get_logger().info(f"計測終了（{why}）")
        self.save(why)

    def _tick(self):
        out = Twist()
        out.linear.z = float(self.nom[2])
        if not self.running:
            out.linear.x, out.linear.y, out.angular.z = (float(self.nom[0]), float(self.nom[1]),
                                                         float(self.nom[3]))
            self.pub.publish(out)
            return
        st = self._state()
        if st is None:
            self.pub.publish(out)
            return self.stop("pose が途切れた")
        x, yaw, v = st
        if np.any(np.abs(self.nom[[0, 1, 3]]) > 1e-6):
            return self.stop("teleop が操作された")
        if np.linalg.norm(x - self.anchor) > self.a.fence:
            return self.stop("フェンスの外")

        now = time.perf_counter()
        while self.seg < len(self.program) and now - self.t_seg >= self.program[self.seg][2]:
            self.seg += 1
            self.t_seg = now
        if self.seg >= len(self.program):
            return self.stop("完了")
        kind, val, _ = self.program[self.seg]
        Rb = self._rot(yaw)
        u_body = np.zeros(2)
        if kind == "hold":
            kcode = 0
            vb = Rb.T @ (self.a.kp_hold * (self.anchor - x))   # 速度指令で位置を保持
            cb = vb
            wz = self.a.kp_yaw * math.atan2(math.sin(self.yaw0 - yaw), math.cos(self.yaw0 - yaw))
        elif kind == "acc":
            kcode = 1
            u_body = np.asarray(val, float)
            # 指令を dt の間一定に保つと  v+ = e v + (1 - e) g c,  e = exp(-dt/tau)。
            # v+ = v + dt u にする c は  c = (v + dt u/(1 - e)) / g - e v/((1 - e) g)
            #                             = v/g + dt u / ((1 - e) g)
            e = np.exp(-self.dt / self.tau)
            vb_now = Rb.T @ v
            cb = (vb_now + self.dt * u_body / (1.0 - e)) / self.gain if self.a.zoh else                 (vb_now + self.tau * u_body) / self.gain
            wz = self.a.kp_yaw * math.atan2(math.sin(self.yaw0 - yaw), math.cos(self.yaw0 - yaw))
        else:
            kcode = 2
            cb = Rb.T @ (self.a.kp_hold * (self.anchor - x))
            wz = float(val)
        cb = np.clip(cb, -self.a.v_cmd_max, self.a.v_cmd_max)
        out.linear.x = float(np.clip(cb[0] / self.a.max_vxy, -1, 1))
        out.linear.y = float(np.clip(cb[1] / self.a.max_vxy, -1, 1))
        out.angular.z = float(np.clip(wz, -1, 1))
        self.pub.publish(out)
        r = self.rec
        r["t"].append(now - self._t0); r["seg"].append(self.seg); r["kind"].append(kcode)
        r["u_body"].append(u_body); r["u_world"].append(Rb @ u_body)
        r["v_est"].append(v); r["x"].append(x); r["yaw"].append(yaw)
        r["cmd"].append([out.linear.x, out.linear.y, out.linear.z, out.angular.z])

    def save(self, why):
        now = datetime.now()
        d = os.path.join(self.a.log_root, "di_excite", now.strftime("%Y-%m-%d"))
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, now.strftime("%H%M%S") + ".npz")
        with self.lock:
            P = np.array(self.poses[max(self.pose_mark - 240, 0):])
        np.savez(path, pose_t=P[:, 0], pose=P[:, 1:],
                 **{k: np.array(v) for k, v in self.rec.items()},
                 dt=self.dt, end_reason=why,
                 segments=str([(k, None if v is None else np.round(v, 3).tolist(), T)
                               for k, v, T in self.program]),
                 params=str(vars(self.a)))
        rate = (len(P) - 1) / max(P[-1, 0] - P[0, 0], 1e-9) if len(P) > 1 else 0
        self.get_logger().info(f"記録保存: {path}（制御 {len(self.rec['t'])} 回, "
                               f"mocap {len(P)} フレーム ≈ {rate:.0f} Hz）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--amps", type=float, nargs="+", default=[0.5, 1.0, 1.3])
    ap.add_argument("--tau-x", type=float, default=0.65)
    ap.add_argument("--tau-y", type=float, default=0.74)
    ap.add_argument("--g-x", type=float, default=1.48)
    ap.add_argument("--g-y", type=float, default=1.52)
    ap.add_argument("--max-vxy", type=float, default=1.0)
    ap.add_argument("--v-cmd-max", type=float, default=1.0)
    ap.add_argument("--kp-hold", type=float, default=0.8)
    ap.add_argument("--kp-yaw", type=float, default=1.0)
    ap.add_argument("--window", type=float, default=0.05)
    ap.add_argument("--no-zoh", dest="zoh", action="store_false",
                    help="連続時間の式 c = (v + tau u)/g を使う（比較用）")
    ap.add_argument("--fence", type=float, default=0.45)
    ap.add_argument("--disp", type=float, default=0.15,
                    help="1回の加速→減速で進む距離の目安 a T^2 [m]（1.4 m 四方のエリアの中央から始めて，模擬で最大 0.32 m）")
    ap.add_argument("--log-root", default=os.path.expanduser("~/research/drone/ros2_ws/log"))
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = DIExcite(args)

    def keys():
        for line in sys.stdin:
            c = line.strip().lower()
            if c == "":
                node.start()
            elif c == "a":
                node.stop("手動で中断")
            elif c == "q":
                node.quit = True
                return

    threading.Thread(target=keys, daemon=True).start()
    try:
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop("終了")
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
