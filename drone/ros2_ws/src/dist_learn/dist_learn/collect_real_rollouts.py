# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機ロールアウト収集（受動レコーダ方式）：CVAE 学習用の (x, u, d) を集める。

SHIELD (IROS 2025) の安全フィルタが使う CVAE は「速度指令 u に対して実際に起きる
状態変化のズレ＝動力学残差 d」の確率分布を学習する。本スクリプトは **実機 Tello** を
**人間がシェアード制御で飛ばし**、モーキャプ(OptiTrack/Motive)で観測した連続2状態から
d を逆算して、学習パイプライン互換の `.npz` に保存する。

■ 設計方針（重要・以前の自律ランダム版から変更）
  狭い実験エリアで自律ランダム飛行は外へ出てしまうため、**人間が keyboard_teleop /
  ジョイスティックで操縦**して全速度域を見せつつエリア内に留める。本スクリプトは
  指令を出さず、**/cmd_vel と /tello/pose を購読するだけの受動レコーダ**。
  記録する u は人間が出した **フィルタ前の生指令**（安全フィルタは収集中 OFF）。
  人間の操縦データも u が生指令である限り d は正しく逆算でき、学習に使える。

────────────────────────────────────────────────────────────────────────────
システムモデル（縮退次元モデル, 論文 式17。公称は単一積分器＝速度入力の一次系）
    x_{k+1} = x_k + Δt·u_k + Δt·d_k
      x = [px, py, θ]   位置 + ヨー角（world座標, Z-up, m/rad）
      u = [vx, vy, ω]   world フレームの速度指令（body ではない！）
      d = [dx, dy, dθ]  動力学残差（CVAE が学習する対象。Tello のゲインずれ・ラグ等を全部吸収）
    d_k = (x_{k+1} - x_k) / Δt_measured - u_k   ← 観測から逆算

────────────────────────────────────────────────────────────────────────────
I/O（既存ノードと同じ規約。drone_control を参照）
    状態源 : /tello/pose (geometry_msgs/PoseStamped)  ← pose_bridge が mocap から流す
             Z-up world, m, yaw = atan2(2(wz+xy), 1-2(yy+zz))
    指令源 : /cmd_vel   (geometry_msgs/Twist)  ← keyboard_teleop / joy が出す人間の指令
             正規化 [-1,1] の **body** 指令。linear.x=前後, linear.y=左右, angular.z=旋回
             tello_sim 規約で full stick 1.0 = MAX_VXY[m/s] 等（下の Config 参照）。
             本スクリプトはこれを購読し、現在 yaw で **body→world に逆変換**して u とする。

■ 制御/サンプリング周期
    指令(Tello)は WiFi 律速で 100Hz は非現実的。**既定 20Hz (Δt=0.05)** でサンプルする。
    deploy 時（安全フィルタ運用）も同じ周期に揃えること（d は周期依存）。
    Δt は **pose 自身のタイムスタンプ差（mocap 実測）** を使う（固定値で割らない）。

────────────────────────────────────────────────────────────────────────────
使い方
  1) インフラ起動（mocap→/tello/pose, /cmd_vel→実機）：
       cd ~/research/drone/ros2_ws && source install/setup.bash
       ros2 launch drone_control bringup_real.launch.py \\
            rigid_body_id:=1 drone_ip:=192.168.11.50
  2) 操縦ノードを起動（人間が飛ばす。T=離陸 L=着陸）：
       ros2 run drone_control keyboard_teleop      # or joy 系
  3) 収集スクリプトを起動（離陸後に開始するのがおすすめ）：
       ros2 run dist_learn collect_real_rollouts \\
            --out ~/research/data/real_rollouts.npz --rate 20
     エリア端に寄ったら手で中心へ戻す。途切れ/着陸は自動でロールアウト境界になる。
     Ctrl-C で安全停止＆それまでのデータを保存。
  4) 達成周期/到着レートだけ先に測る（飛ばさず観測）：
       ros2 run dist_learn collect_real_rollouts --measure-rate --duration 10
  5) 収集データを可視化（実測 vs 理論一次系、残差分布、時系列）：
       ros2 run dist_learn plot_rollouts --data ~/research/data/real_rollouts.npz
     収集と同時に出すなら collect 側に --plot を付ける。
  6) 学習（既存パイプラインを流用。本スクリプトは学習コードを書かない）：
       python train_cvae.py --data ~/research/data/real_rollouts.npz
  7) ハード無しの単体テスト（モックに既知外乱を注入して逆算精度を検証）：
       python collect_real_rollouts.py --self-test

  ■ Isaac sim での動作確認（実機の前のエンドツーエンド検証）
    本スクリプトはトピック互換なので、実機の代わりに tello_sim.py をつなげば
    そのまま sim データを収集できる：
       source ~/env_isaaclab/bin/activate && python ~/research/drone/sim/tello_sim.py
       ros2 run drone_control keyboard_teleop          # 人間が操縦
       ros2 run dist_learn collect_real_rollouts --plot # 収集＋可視化
    注意：d は 0 にならない。tello_sim は SMOOTH=0.1 の一次遅れ（指令速度に
    ラグ追従）と壁クリップを持つので、公称の単一積分器との差が d に出る。
    逆に SMOOTH=1.0（遅れ無し）・壁から離れて飛べば d≈0（数値/タイミング誤差のみ）に
    なるはず → パイプライン健全性のチェックに使える。yaw だけは sim が角速度を直接
    与えるため dθ≈0 になりやすい。

────────────────────────────────────────────────────────────────────────────
出力 .npz のキー
    x            (N,3)  [px,py,θ]   各遷移の **前** の状態
    u            (N,3)  [vx,vy,ω]   その遷移中に出ていた生（unfiltered）指令（world）
    d            (N,3)  [dx,dy,dθ]  逆算した残差
    rollout_starts (R,) int         連続区間（ロールアウト）先頭の行インデックス
    dt           (N,)   float       各遷移の実測 Δt（タイミング監査用）
"""

from __future__ import annotations

import argparse
import math
import os
import time
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


# =============================================================================
# 角度ユーティリティ（±π境界で破綻しない差分）
# =============================================================================
def wrap_angle(a: float) -> float:
    """角を (-π, π] に正規化。"""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a: float, b: float) -> float:
    """a - b を最短角で。dθ_raw = atan2(sin(a-b), cos(a-b))。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """クォータニオン → ヨー角[rad]（Zまわり）。position_controller と同式。"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def body_cmd_to_world_u(lin_x: float, lin_y: float, ang_z: float,
                        yaw: float, cfg: "Config") -> np.ndarray:
    """正規化 body 指令 (/cmd_vel) → world 速度 u=[vx,vy,ω][m/s,rad/s]。

    body→world は position_controller / tello_sim と同じ回転規約。
    ω はヨーレートで 2D なので body/world 同一。
    """
    bvx = lin_x * cfg.max_vxy
    bvy = lin_y * cfg.max_vxy
    wz = ang_z * cfg.max_wz
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([cy * bvx - sy * bvy, sy * bvx + cy * bvy, wz])


# =============================================================================
# 設定
# =============================================================================
@dataclass
class Config:
    # --- サンプリング周期（既定 20Hz。WiFi 律速で 100Hz は非現実的）---
    rate_hz: float = 20.0

    # --- /cmd_vel 正規化スケール（tello_sim 規約。実機の最大速度に合わせ調整）---
    max_vxy: float = 1.0     # full stick = この m/s
    max_wz: float = 3.0      # full stick = この rad/s

    # --- データ品質フィルタ（連続性チェック）---
    dt_min_frac: float = 0.4  # 実測 Δt が公称dt*この値 未満 → 重複/異常で除外＆境界
    dt_max_frac: float = 3.0  # 〃 を超える → ドロップ/着陸ギャップで除外＆境界
    d_clip: float = 30.0      # |d| 成分がこれを超える遷移は除外＆境界（mocap dropout）

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz


# =============================================================================
# I/O アダプタ（抽象）。ハード無しでもモックで差し替えできる。
# =============================================================================
class RolloutIO:
    """状態・指令の購読を抽象化。ROS2 実装とモック実装が継承する。"""

    def get_state(self) -> Optional[Tuple[float, np.ndarray]]:
        """最新の (pose_stamp[s], x=[px,py,θ])。未受信なら None。

        pose_stamp は pose 自身のタイムスタンプ（mocap 実測時刻）。Δt はこの差分。
        """
        raise NotImplementedError

    def get_cmd(self) -> np.ndarray:
        """最新の /cmd_vel を正規化 body [lin_x, lin_y, ang_z] で返す。未受信は零。"""
        raise NotImplementedError

    def spin(self) -> None:
        """受信処理を進める（ROS2 は bg スレッドなので no-op／モックは1step進める）。"""

    def shutdown(self) -> None:
        pass


# =============================================================================
# ROS2 アダプタ（実機・受動レコーダ）：購読のみ、publish しない
# =============================================================================
class Ros2IO(RolloutIO):
    def __init__(self, cfg: Config):
        import threading

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped, Twist

        self._rclpy = rclpy
        self._lock = threading.Lock()
        self._latest_pose: Optional[Tuple[float, np.ndarray]] = None
        self._latest_cmd = np.zeros(3)
        self._pose_count = 0
        self._cmd_count = 0

        rclpy.init()
        self.node = rclpy.create_node("collect_real_rollouts")
        self.node.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 10)
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self.node.get_logger().info(
            "collect_real_rollouts up（受動レコーダ）. sub:/tello/pose, /cmd_vel"
        )

    def _on_pose(self, msg) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest_pose = (stamp, np.array([p.x, p.y, yaw], dtype=float))
            self._pose_count += 1

    def _on_cmd(self, msg) -> None:
        with self._lock:
            self._latest_cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
            self._cmd_count += 1

    def get_state(self):
        with self._lock:
            if self._latest_pose is None:
                return None
            stamp, x = self._latest_pose
            return stamp, x.copy()

    def get_cmd(self) -> np.ndarray:
        with self._lock:
            return self._latest_cmd.copy()

    def counts(self) -> Tuple[int, int]:
        with self._lock:
            return self._pose_count, self._cmd_count

    def shutdown(self) -> None:
        self._executor.shutdown()
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# =============================================================================
# モックアダプタ（ハード無しの単体テスト用）
# =============================================================================
class MockIO(RolloutIO):
    """「人間操縦」を OU 過程で模擬し、既知外乱で動く偽の機体。逆算精度を検証する。

    真のモデル： x_{k+1} = x_k + dt·(u_world + d_true),  d_true = bias·u_world + noise
      pending_cmd（次に作用させる body 指令）を get_cmd で公開し、spin() で
      pending を作用→新 pending をサンプル。collector の読み順と整合させてある。
    """

    def __init__(self, cfg: Config, bias: float = -0.15, noise_std: float = 0.02,
                 seed: int = 0):
        self.cfg = cfg
        self.bias = bias
        self.noise_std = noise_std
        self.rng = np.random.default_rng(seed)
        self.x = np.array([0.0, 0.0, 0.0])
        self.t = 0.0
        self.pending = self._sample_cmd()          # 次に作用させる body 指令
        self.d_true_log: List[np.ndarray] = []
        self._steps = 0

    def _sample_cmd(self) -> np.ndarray:
        # OU 風に滑らかに動く「人間っぽい」正規化 body 指令
        if not hasattr(self, "_c"):
            self._c = np.zeros(3)
        self._c = 0.92 * self._c + 0.08 * self.rng.uniform(-1, 1, 3)
        return np.clip(self._c, -1, 1)

    def spin(self) -> None:
        c = self.cfg
        u_world = body_cmd_to_world_u(self.pending[0], self.pending[1],
                                      self.pending[2], self.x[2], c)
        d_true = self.bias * u_world + self.rng.normal(0.0, self.noise_std, 3)
        self.x = self.x + c.dt * (u_world + d_true)
        self.x[2] = wrap_angle(self.x[2])
        self.t += c.dt
        self.d_true_log.append(d_true.copy())
        self.pending = self._sample_cmd()
        self._steps += 1

    def get_state(self):
        return self.t, self.x.copy()

    def get_cmd(self) -> np.ndarray:
        return self.pending.copy()


# =============================================================================
# 収集本体
# =============================================================================
class Collector:
    def __init__(self, io: RolloutIO, cfg: Config):
        self.io = io
        self.cfg = cfg
        self.all_x: List[np.ndarray] = []
        self.all_u: List[np.ndarray] = []
        self.all_d: List[np.ndarray] = []
        self.all_dt: List[float] = []
        self.rollout_starts: List[int] = []
        self.n_dropped = 0
        self._pending_break = True   # 次の有効遷移は新ロールアウト先頭

    def _wait_first_pose(self, timeout_s: float = 10.0):
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout_s:
            self.io.spin()
            st = self.io.get_state()
            if st is not None:
                return st
            time.sleep(0.02)
        raise RuntimeError("pose を受信できない（mocap/pose_bridge は動いてる？）")

    def _add_transition(self, x_prev, u_prev, x_cur, dt) -> None:
        """遷移 (x_prev,u_prev)→x_cur から d を逆算し蓄積。異常は除外し境界を立てる。"""
        c = self.cfg
        # 連続性チェック：Δt 窓を外れる＝重複/ドロップ/着陸ギャップ → 不連続
        if not (c.dt * c.dt_min_frac <= dt <= c.dt * c.dt_max_frac):
            self.n_dropped += 1
            self._pending_break = True
            return
        d = np.empty(3)
        d[0] = (x_cur[0] - x_prev[0]) / dt - u_prev[0]
        d[1] = (x_cur[1] - x_prev[1]) / dt - u_prev[1]
        d[2] = angle_diff(x_cur[2], x_prev[2]) / dt - u_prev[2]  # θは最短角
        if np.any(np.abs(d) > c.d_clip):
            self.n_dropped += 1
            self._pending_break = True
            return
        if self._pending_break:
            self.rollout_starts.append(len(self.all_x))
            self._pending_break = False
        self.all_x.append(x_prev.copy())
        self.all_u.append(u_prev.copy())
        self.all_d.append(d)
        self.all_dt.append(dt)

    def run(self, duration_s: Optional[float] = None, realtime: bool = True) -> None:
        c = self.cfg
        print(f"[collect] 受動レコーダ起動: rate={c.rate_hz}Hz (Δt={c.dt:.4f}) "
              f"{'duration=%.1fs' % duration_s if duration_s else '無制限(Ctrl-Cで停止)'}")
        print("[collect] keyboard_teleop 等で人間が操縦してください。記録中 …")
        self._wait_first_pose()

        prev: Optional[Tuple[float, np.ndarray, np.ndarray]] = None  # (t, x, u_world)
        next_t = time.perf_counter()
        t_start = next_t
        max_steps = None if duration_s is None else int(duration_s / c.dt)
        step = 0
        try:
            while True:
                if max_steps is not None and step >= max_steps:
                    break
                step += 1
                self.io.spin()
                st = self.io.get_state()
                if st is not None:
                    t, x = st
                    # 今の body 指令を現在 yaw で world u に逆変換
                    cmd = self.io.get_cmd()
                    u_world = body_cmd_to_world_u(cmd[0], cmd[1], cmd[2], x[2], c)
                    if prev is not None:
                        t_prev, x_prev, u_prev = prev
                        if t > t_prev:                       # 新 pose のときだけ遷移化
                            self._add_transition(x_prev, u_prev, x, t - t_prev)
                        # 同一 stamp（mocap 未更新）は捨てて prev 据え置き
                    prev = (t, x, u_world)
                next_t += c.dt
                if realtime:
                    self._sleep_until(next_t)
        except KeyboardInterrupt:
            print("\n[collect] Ctrl-C 検出。保存します。")

    @staticmethod
    def _sleep_until(deadline: float) -> None:
        dt = deadline - time.perf_counter()
        if dt > 0:
            time.sleep(dt)

    def save_and_report(self, out_path: str) -> dict:
        if not self.all_x:
            print("[collect] 記録された遷移が 0 件。保存しない。")
            return {}
        x = np.array(self.all_x)
        u = np.array(self.all_u)
        d = np.array(self.all_d)
        dt = np.array(self.all_dt)
        starts = np.array(self.rollout_starts, dtype=int)
        np.savez(out_path, x=x, u=u, d=d, rollout_starts=starts, dt=dt)
        report = quality_report(x, u, d, dt, self.n_dropped, starts)
        print(f"\n[collect] 保存: {out_path}  (N={len(x)} 遷移, {len(starts)} ロールアウト)")
        return report


# =============================================================================
# データ品質チェック
# =============================================================================
def quality_report(x, u, d, dt, n_dropped, starts) -> dict:
    print("\n========== データ品質レポート ==========")
    print(f"遷移数 N = {len(x)} / 除外した異常遷移 = {n_dropped}")
    print(f"ロールアウト境界 rollout_starts = {starts.tolist()}")

    np.set_printoptions(precision=4, suppress=True)
    print(f"\nd.mean(axis=0) = {d.mean(axis=0)}   [dx, dy, dθ]")
    print(f"d.std (axis=0) = {d.std(axis=0)}")
    print(f"u  範囲: vx[{u[:,0].min():+.2f},{u[:,0].max():+.2f}] "
          f"vy[{u[:,1].min():+.2f},{u[:,1].max():+.2f}] "
          f"ω[{u[:,2].min():+.2f},{u[:,2].max():+.2f}]  (励起された速度域)")

    print(f"\nΔt[ms]: mean={dt.mean()*1e3:.3f} std={dt.std()*1e3:.3f} "
          f"min={dt.min()*1e3:.3f} max={dt.max()*1e3:.3f}")
    edges = np.linspace(dt.min(), dt.max(), 9)
    hist, _ = np.histogram(dt, bins=edges)
    for i in range(len(hist)):
        bar = "#" * int(40 * hist[i] / max(hist.max(), 1))
        print(f"  [{edges[i]*1e3:6.2f},{edges[i+1]*1e3:6.2f}] ms | {hist[i]:6d} {bar}")

    print("\nE[d|u] 線形傾き（d_i ≈ slope_i · u_i, 切片込み最小二乗）:")
    labels = ["x", "y", "θ"]
    slopes = []
    for i in range(3):
        ui, di = u[:, i], d[:, i]
        if np.ptp(ui) < 1e-9:
            slopes.append(0.0)
            print(f"  {labels[i]}: u がほぼ一定で推定不可")
            continue
        A = np.vstack([ui, np.ones_like(ui)]).T
        slope, intercept = np.linalg.lstsq(A, di, rcond=None)[0]
        slopes.append(float(slope))
        note = "（アンダーシュート）" if slope < -0.02 else ""
        print(f"  {labels[i]}: slope={slope:+.4f} intercept={intercept:+.4f} {note}")

    return {
        "N": int(len(x)),
        "n_dropped": int(n_dropped),
        "d_mean": d.mean(axis=0),
        "d_std": d.std(axis=0),
        "slopes": np.array(slopes),
        "dt_mean": float(dt.mean()),
    }


# =============================================================================
# 周期/到着レート実測（飛ばさず観測するドライラン）
# =============================================================================
def measure_rate(io: "Ros2IO", cfg: Config, duration_s: float) -> None:
    print(f"[measure] {duration_s:.1f}s 間 /tello/pose・/cmd_vel の到着レートと "
          f"ループ周期を観測（指令は出さない）")
    loop_dt: List[float] = []
    pose_stamps: List[float] = []
    p0, c0 = io.counts()
    t0 = time.perf_counter()
    next_t = t0
    last = t0
    last_stamp = None
    while time.perf_counter() - t0 < duration_s:
        st = io.get_state()
        if st is not None and st[0] != last_stamp:
            pose_stamps.append(st[0])
            last_stamp = st[0]
        now = time.perf_counter()
        loop_dt.append(now - last)
        last = now
        next_t += cfg.dt
        Collector._sleep_until(next_t)
    p1, c1 = io.counts()
    dur = time.perf_counter() - t0
    print(f"[measure] /tello/pose: {(p1-p0)/dur:6.1f} Hz 受信")
    print(f"[measure] /cmd_vel  : {(c1-c0)/dur:6.1f} Hz 受信"
          + ("  （0Hz=操縦ノード未起動?）" if c1 == c0 else ""))
    if len(pose_stamps) > 2:
        ps = np.diff(np.array(pose_stamps))
        print(f"[measure] pose stamp Δt[ms]: mean={ps.mean()*1e3:.2f} "
              f"std={ps.std()*1e3:.2f} min={ps.min()*1e3:.2f} max={ps.max()*1e3:.2f}")
    ld = np.array(loop_dt[1:])
    print(f"[measure] loop Δt[ms]: mean={ld.mean()*1e3:.2f} std={ld.std()*1e3:.2f} "
          f"（目標 {cfg.dt*1e3:.1f}ms）")


# =============================================================================
# 単体テスト（モック）：注入 d_true を復元できるか
# =============================================================================
def self_test() -> int:
    print("===== self-test: モック運動学で収集ロジックを検証 =====")
    cfg = Config(rate_hz=20.0)
    bias, noise = -0.15, 0.02
    io = MockIO(cfg, bias=bias, noise_std=noise, seed=1)
    col = Collector(io, cfg)
    col.run(duration_s=cfg.dt * 6000, realtime=False)   # 6000 ステップ・実時間sleepなし
    rep = col.save_and_report("/tmp/selftest_rollouts.npz")

    slopes = rep["slopes"]
    ok = True
    for i, name in enumerate(["x", "y", "θ"]):
        err = abs(slopes[i] - bias)
        verdict = "OK" if err < 0.03 else "NG"
        if err >= 0.03:
            ok = False
        print(f"  slope[{name}]={slopes[i]:+.4f} vs bias={bias:+.4f} "
              f"|err|={err:.4f} -> {verdict}")
    print(f"  注入 d_true 件数={len(io.d_true_log)}  記録 d 件数={rep['N']}")
    print("\n===== self-test:", "PASS =====" if ok else "FAIL =====")
    return 0 if ok else 1


# =============================================================================
# CLI
# =============================================================================
def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="実機ロールアウト収集（受動レコーダ・人間シェアード制御）")
    p.add_argument("--out", default=None,
                   help="出力 .npz パス（明示時はそのまま使う）。未指定なら "
                        "<log-dir>/<日付>/<時刻>_rollouts.npz に自動保存")
    p.add_argument("--log-dir", default="log/residual",
                   help="自動保存のルート（既定 ./log/residual）。配下に日付フォルダを作る")
    p.add_argument("--rate", type=float, default=20.0, help="サンプリング周期[Hz]（既定20）")
    p.add_argument("--max-vxy", type=float, default=1.0, help="full stick の m/s（正規化解除）")
    p.add_argument("--max-wz", type=float, default=3.0, help="full stick の rad/s")
    p.add_argument("--duration", type=float, default=None, help="自動停止する秒数（既定:無制限）")
    p.add_argument("--measure-rate", action="store_true", help="到着レート/周期だけ実測して終了")
    p.add_argument("--plot", action="store_true", help="保存後に実測vs理論のグラフを表示")
    p.add_argument("--self-test", action="store_true", help="ハード無しのモック検証")
    args = p.parse_args(argv)

    if args.self_test:
        return self_test()

    # 出力先：明示が無ければ log/residual/<日付>/<時刻>_rollouts.npz を作る
    out_path = args.out
    if out_path is None:
        now = datetime.now()
        out_dir = os.path.join(args.log_dir, now.strftime("%Y-%m-%d"))
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, now.strftime("%H%M%S") + "_rollouts.npz")
    else:
        d = os.path.dirname(out_path)
        if d:
            os.makedirs(d, exist_ok=True)
    print(f"[collect] 保存先: {os.path.abspath(out_path)}")

    cfg = Config(rate_hz=args.rate, max_vxy=args.max_vxy, max_wz=args.max_wz)
    io = Ros2IO(cfg)
    try:
        if args.measure_rate:
            measure_rate(io, cfg, args.duration or 10.0)
            return 0
        col = Collector(io, cfg)
        try:
            col.run(duration_s=args.duration)
        finally:
            rep = col.save_and_report(out_path)
            if args.plot and rep:
                try:
                    from dist_learn.plot_rollouts import make_figures
                    make_figures(np.load(out_path), show=True,
                                 out_prefix=out_path.rsplit(".", 1)[0])
                except Exception as e:
                    print(f"[collect] グラフ生成に失敗: {e}")
    finally:
        io.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
