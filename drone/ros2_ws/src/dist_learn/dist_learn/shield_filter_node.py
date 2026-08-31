# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""SHIELD 安全フィルタ ROS2 ノード（単一障害物・S-DTCBF）。

学習済み CVAE（cvae_real.pth）と SHIELD の確率的バリア(S-DTCBF)を使い、
**生の速度指令 /cmd_vel_nom を安全な /cmd_vel に整形して**機体へ流す中間ノード。
アルゴリズム本体は再実装せず、ユーザーの再現リポジトリ(~/202605_SHIELD)の
`shield.safety_filter.SHIELDFilter` をそのまま import して使う。

    keyboard_teleop / position_controller
        │ /cmd_vel_nom (Twist, body正規化)   ← 生指令（フィルタ前）
        ▼
    shield_filter_node
        ├ /tello/pose 購読 → x=[px,py], yaw
        ├ u_nom を body→world 速度へ変換
        ├ 直近4の (x, u_nom) で CVAE 文脈を作る（★生指令を保存：algebraic loop回避）
        ├ SHIELDFilter.filter(x, u_nom, obstacles, r, ctx_xs, ctx_us) → u_safe(world)
        └ u_safe を world→body正規化して publish（z/旋回は素通し）
        │ /cmd_vel (Twist, body正規化)        ← フィルタ後＝安全
        ▼  tello_driver → 実機 / tello_sim

■ モデル規約（収集・学習と必ず一致させること）
  - 2D 位置 x=[px,py]（yaw はバリアに使わない。旋回指令は素通し）
  - u=[vx,vy] world 速度。/cmd_vel(body正規化) と現在 yaw で相互変換
  - dt は収集・学習と同じ **20Hz(0.05)**（フィルタ制約は dt 依存）

■ 起動（重要：torch を使うので env_isaaclab の python で直起動する）
  `ros2 run` は system python を使い torch が無いので不可。代わりに:

    source /opt/ros/humble/setup.bash
    source ~/research/drone/ros2_ws/install/setup.bash
    ~/env_isaaclab/bin/python \\
      ~/research/drone/ros2_ws/src/dist_learn/dist_learn/shield_filter_node.py \\
      --ros-args \\
        -p obstacles:="[1.0, 0.0, 0.3]" \\
        -p robot_radius:=0.15 -p u_max:=0.5

  操縦ノードは生指令を /cmd_vel_nom に出すようリマップする:
    ros2 run drone_control keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_nom

  まず sim で検証（tello_sim.py + 仮想障害物）→ 問題なければ実機。
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class ShieldFilterNode(Node):
    def __init__(self):
        super().__init__("shield_filter_node")

        # --- パラメータ ---
        self.declare_parameter("mode", "shield")  # "shield"(CVAE) or "nominal"(素のCBF)
        self.declare_parameter("shield_path",
                               os.path.expanduser("~/202605_SHIELD/shield_reproduction"))
        self.declare_parameter("cvae_path",
                               os.path.expanduser(
                                   "~/202605_SHIELD/shield_reproduction/models/cvae_real.pth"))
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_vxy", 1.0)   # /cmd_vel 正規化スケール（tello_sim規約）
        self.declare_parameter("max_wz", 3.0)
        # 障害物：[x1,y1,r1, x2,y2,r2, ...] のフラット配列。単一障害物なら3要素。
        self.declare_parameter("obstacles", [1.0, 0.0, 0.3])
        # mocap 由来の障害物位置を購読する topic（""=無効＝上の固定値を使う）。
        # obstacle_bridge が同じ [x,y,r] フラット配列(Float32MultiArray)で流す。
        self.declare_parameter("obstacle_topic", "")
        self.declare_parameter("robot_radius", 0.15)
        self.declare_parameter("u_max", 0.5)       # フィルタ出力の上限[m/s]
        self.declare_parameter("fixed_alpha", 0.9)  # Phase4: 固定α（<0 で動的αに）
        self.declare_parameter("lam", 10.0)
        self.declare_parameter("gamma", 0.1)
        self.declare_parameter("P", 0.05)
        self.declare_parameter("K", 10)
        self.declare_parameter("delta", 1.0)
        self.declare_parameter("n_samples", 200)
        self.declare_parameter("context_len", 4)
        # shield_multi 用：LSEスムースminの鋭さ beta と分散補正の精緻化フラグ
        self.declare_parameter("beta", 1.0)          # 大きいほど min に近い（鋭い）
        self.declare_parameter("use_refined", True)  # True=Thm5.2 / False=Prop5.1(保守的)
        # --- QP スラック変数（デッドロックでも入力を出す）---
        # CBF制約 a^T u >= b を a^T u >= b - s に緩め、コストへ w*s^2 を足す。
        # 硬い制約のままだと必要な |u| が u_max を超えた瞬間に clip で“黙って違反”し、
        # かつ u の全予算が退避方向に食われてゴール方向成分が消える＝デッドロック。
        # w を有限にすると緩みが最小限で明示的になり、進行方向の予算が残る。
        #   目安: 100以上≒ほぼ硬い / 10前後=デッドロックが解ける / 1=かなり緩い
        #   <0 で無効＝従来どおりの硬い制約（既定）
        self.declare_parameter("slack_weight", -1.0)
        # 緩めていい上限[m/s]（s <= slack_max）。<0 で無制限。安全側の歯止め。
        self.declare_parameter("slack_max", -1.0)
        # --- 会場境界の壁（領域外＝mocap範囲外に出さない）---
        # 壁は「巨大円の障害物」で近似して既存の h_hat_multi / LSE smooth-min に相乗りする
        # （barrier.py に半平面バリアを足さないので凹性の前提もそのまま）。
        self.declare_parameter("wall_enable", False)
        # [x_min, x_max, y_min, y_max][m]。機体“中心”は各辺から robot_radius 内側までに制限。
        self.declare_parameter("wall_bounds", [-1.0, 1.0, -1.0, 1.0])
        # 壁を近似する円の半径[m]。大きいほど直線に近い（会場端での誤差 ~ 半幅^2/(2R)）。
        # 実測（会場±1m, robot_r=0.10）: R=10m→49mm, R=50m→10mm, R=200m→2.5mm。
        self.declare_parameter("wall_arc_radius", 200.0)
        self.declare_parameter("pass_through_z", True)   # 高さ指令を素通し
        self.declare_parameter("pass_through_yaw", True)  # 旋回指令を素通し
        self.declare_parameter("world_frame", "world")    # マーカーの座標系（/tello/pose と同じ）
        self.declare_parameter("marker_height", 1.5)      # RViz円柱の高さ[m]（見やすさ用）
        # 軌跡・h の記録（後からプロット用）。log/traj/<日付>/<時刻>_<mode>.npz に保存
        self.declare_parameter("record", True)
        self.declare_parameter("log_root",
                               os.path.expanduser("~/research/drone/ros2_ws/log"))

        self.max_vxy = float(self.get_parameter("max_vxy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.context_len = int(self.get_parameter("context_len").value)
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.pass_z = bool(self.get_parameter("pass_through_z").value)
        self.pass_yaw = bool(self.get_parameter("pass_through_yaw").value)

        # --- SHIELD リポジトリを import path に追加してフィルタを構築 ---
        shield_path = self.get_parameter("shield_path").value
        if shield_path not in sys.path:
            sys.path.insert(0, shield_path)
        from shield.barrier import BarrierFunction
        from shield.env import Obstacle
        from shield.safety_filter import SHIELDFilter, NominalDTCBF, SHIELDMultiFilter

        self._Obstacle = Obstacle
        self.mode = str(self.get_parameter("mode").value).lower()
        if self.mode not in ("shield", "shield_multi", "nominal"):
            raise ValueError(
                f"mode は 'shield' / 'shield_multi' / 'nominal': {self.mode}")

        # 障害物リスト構築。
        # obstacle_topic 指定時は mocap からの最初の受信まで「障害物なし」で開始する
        # （静的パラメータ obstacles は使わない＝意図しない既定障害物 (1,0) を出さない）。
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        if self.obstacle_topic:
            self.obstacles = []
        else:
            self.obstacles = self._build_obstacles(
                list(self.get_parameter("obstacles").value)
            )

        # 会場境界の壁（mocap 由来の障害物とは別に持つ。マーカーは箱として描く）
        self.wall_enable = bool(self.get_parameter("wall_enable").value)
        self.wall_bounds = [float(v) for v in self.get_parameter("wall_bounds").value]
        if len(self.wall_bounds) != 4:
            raise ValueError(
                f"wall_bounds は [x_min, x_max, y_min, y_max] の4要素: {self.wall_bounds}")
        self.wall_arc_radius = float(self.get_parameter("wall_arc_radius").value)
        self.walls = self._build_walls() if self.wall_enable else []

        barrier = BarrierFunction(lam=float(self.get_parameter("lam").value),
                                  gamma=float(self.get_parameter("gamma").value))
        u_max = float(self.get_parameter("u_max").value)
        fa = float(self.get_parameter("fixed_alpha").value)
        # スラック：負値は「無効」を表すセンチネル。None を渡すと従来の硬い制約になる。
        sw = float(self.get_parameter("slack_weight").value)
        sm = float(self.get_parameter("slack_max").value)
        self.slack_weight = sw if sw >= 0.0 else None
        self.slack_max = sm if sm >= 0.0 else float("inf")
        self._info = ""

        if self.mode in ("shield", "shield_multi"):
            # 学習済み CVAE を使う確率的バリア（S-DTCBF）。トラッキング補正(u-μ_d)あり。
            from shield.cvae import CVAE
            import torch
            cvae = CVAE(d_dim=2, state_dim=2, action_dim=2,
                        context_len=self.context_len, latent_dim=3, hidden=64)
            cvae_path = self.get_parameter("cvae_path").value
            cvae.load_state_dict(torch.load(cvae_path, map_location="cpu"))
            cvae.eval()
            alpha_lbl = ("fixed %.2f" % fa) if fa >= 0 else "dynamic"
            if self.mode == "shield_multi":
                # 全障害物を同時に考慮（h_hat_multi）。最近傍だけでなく挟まれも守る。
                self.filter = SHIELDMultiFilter(
                    barrier=barrier, cvae=cvae,
                    beta=float(self.get_parameter("beta").value),
                    P=float(self.get_parameter("P").value),
                    K=int(self.get_parameter("K").value),
                    delta=float(self.get_parameter("delta").value),
                    dt=self.dt, u_max=u_max,
                    n_samples=int(self.get_parameter("n_samples").value),
                    fixed_alpha=(fa if fa >= 0.0 else None),
                    use_refined=bool(self.get_parameter("use_refined").value),
                    slack_weight=self.slack_weight, slack_max=self.slack_max,
                )
                self._info = (f"SHIELD-Multi(CVAE) alpha={alpha_lbl} "
                              f"beta={self.get_parameter('beta').value}")
            else:
                # 最近傍1障害物のみを考慮する素のSHIELD。
                self.filter = SHIELDFilter(
                    barrier=barrier, cvae=cvae,
                    P=float(self.get_parameter("P").value),
                    K=int(self.get_parameter("K").value),
                    delta=float(self.get_parameter("delta").value),
                    dt=self.dt, u_max=u_max,
                    n_samples=int(self.get_parameter("n_samples").value),
                    fixed_alpha=(fa if fa >= 0.0 else None),
                    slack_weight=self.slack_weight, slack_max=self.slack_max,
                )
                self._info = f"SHIELD(CVAE) alpha={alpha_lbl}"
        else:
            # 素のCBF：外乱モデル無し(d=0)・補正無し。無入力なら出力ゼロ＝ドリフトしない。
            self.filter = NominalDTCBF(
                barrier=barrier,
                alpha=(fa if fa >= 0.0 else 0.9),
                dt=self.dt, u_max=u_max,
                slack_weight=self.slack_weight, slack_max=self.slack_max,
            )
            self._info = f"NominalCBF(d=0) alpha={(fa if fa >= 0 else 0.9):.2f}"

        # --- 状態 ---
        self.have_pose = False
        self.pos = np.zeros(2)   # [px, py]
        self.yaw = 0.0
        self.u_nom_body = np.zeros(4)  # [lin_x, lin_y, lin_z, ang_z]（生指令）
        # 文脈バッファ（直近 context_len の (x, u_world) ペア）
        self.ctx_x = deque([np.zeros(2)] * self.context_len, maxlen=self.context_len)
        self.ctx_u = deque([np.zeros(2)] * self.context_len, maxlen=self.context_len)

        # --- I/O ---
        self.create_subscription(PoseStamped, "/tello/pose", self._on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel_nom", self._on_cmd_nom, 10)
        # mocap 由来の障害物位置（任意）。指定時は毎メッセージで self.obstacles を更新
        if self.obstacle_topic:
            from std_msgs.msg import Float32MultiArray
            self.create_subscription(
                Float32MultiArray, self.obstacle_topic, self._on_obstacles, 10)
            self.get_logger().info(
                f"障害物を mocap topic から購読: {self.obstacle_topic}（受信まで障害物なし）")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(self.dt, self._tick)

        # RViz マーカー：late-join の RViz でも見えるよう latched(transient_local) で出す
        marker_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.marker_pub = self.create_publisher(MarkerArray, "/shield/obstacles", marker_qos)
        self.world_frame = self.get_parameter("world_frame").value
        self.marker_height = float(self.get_parameter("marker_height").value)
        self._publish_markers()
        self.create_timer(2.0, self._publish_markers)  # 念のため定期再送

        # ライブ軌跡（RViz の Path 表示用）
        self.path_pub = self.create_publisher(Path, "/shield/trajectory", 10)
        self._path = Path()
        self._path.header.frame_id = self.world_frame

        # 記録バッファ（後処理プロット用）
        self.record = bool(self.get_parameter("record").value)
        self.log_root = self.get_parameter("log_root").value
        self._t0 = time.perf_counter()
        self._rec = {k: [] for k in
                     ("t", "px", "py", "yaw", "unom", "usafe", "h", "sdf", "alpha",
                      "slack")}
        self._slack_steps = 0      # スラックが効いた（s>0）ステップ数
        self._slack_peak = 0.0     # そのピーク[m/s]

        obs_str = ", ".join(f"({o.position[0]:.2f},{o.position[1]:.2f},r={o.radius:.2f})"
                            for o in self.obstacles)
        slack_str = ("硬い制約(slack無効)" if self.slack_weight is None else
                     f"slack w={self.slack_weight:g} max="
                     f"{'inf' if self.slack_max == float('inf') else f'{self.slack_max:g}'}")
        if self.wall_enable:
            xn, xp, yn, yp = self.wall_bounds
            wall_str = (f"壁 x∈[{xn:+.2f},{xp:+.2f}] y∈[{yn:+.2f},{yp:+.2f}] "
                        f"(機体中心の限界は各辺 -{self.robot_radius:.2f}m, arc_R="
                        f"{self.wall_arc_radius:g}m)")
        else:
            wall_str = "壁なし"
        self.get_logger().info(
            f"shield_filter_node up [{self._info}]. dt={self.dt:.3f} "
            f"robot_r={self.robot_radius} u_max={self.filter.u_max} | "
            f"obstacles=[{obs_str}] | {wall_str} | {slack_str} | "
            f"sub:/tello/pose,/cmd_vel_nom pub:/cmd_vel"
        )

    def _build_walls(self):
        """会場境界の4辺を「巨大円の障害物」として作る（半平面バリアの近似）。

        なぜ円で近似するか:
            barrier.py に半平面バリアを新設せず、既存の h_hat / h_hat_multi・LSE
            smooth-min・Prop4.2の凹性・Thm5.2の分散補正の経路にそのまま相乗りできる。
            壁 x = x_max は「中心 (x_max + Rw, 0)・半径 Rw の円」で近似する:
                h = |p - c| - Rw - robot_r ≈ (x_max - px) - robot_r      (Rw >> 会場半幅)
            つまり機体中心は x <= x_max - robot_r に保たれる。
            円弧のふくらみによる誤差は会場端で ~ (半幅)^2 / (2 Rw)
            （Rw=50m・半幅1m なら約1cm。h を過大評価＝わずかに甘い側）。

        注意（LSE smooth-min のバイアス）:
            h_hat_multi は障害物 N 個が同程度の h のとき min より log(N)/beta だけ
            低く出る。壁4枚を足すと N が 4 増えるので b の要求 |u| が上がり、u_max が
            小さいと制約を満たせなくなる。beta を上げる（例 1.0→3〜4）と log(N)/beta が
            縮んで解消する。lambda_max_multi=lam*gamma^2*(1+beta*lam) 経由で分散補正が
            増えるが、ここでは dt^2 が掛かるので実質無視できる大きさ。
        """
        x_min, x_max, y_min, y_max = self.wall_bounds
        Rw = self.wall_arc_radius
        specs = [
            (np.array([x_max + Rw, 0.0]),),   # x = x_max の壁
            (np.array([x_min - Rw, 0.0]),),   # x = x_min の壁
            (np.array([0.0, y_max + Rw]),),   # y = y_max の壁
            (np.array([0.0, y_min - Rw]),),   # y = y_min の壁
        ]
        return [self._Obstacle(position=c, radius=Rw) for (c,) in specs]

    def _all_obstacles(self):
        """フィルタ・バリア評価に渡す障害物（mocap由来 + 会場境界の壁）。"""
        return self.obstacles + self.walls

    def _arena_markers(self):
        """会場境界（実線）と機体中心の限界（内側の細線）を RViz に出す。"""
        if not self.wall_enable:
            return []
        x_min, x_max, y_min, y_max = self.wall_bounds
        r = self.robot_radius
        out = []
        for i, (box, width, rgba) in enumerate((
            ((x_min, x_max, y_min, y_max), 0.02, (0.35, 0.35, 0.35, 0.9)),
            ((x_min + r, x_max - r, y_min + r, y_max - r), 0.01, (1.0, 0.4, 0.7, 0.5)),
        )):
            xn, xp, yn, yp = box
            m = Marker()
            m.header.frame_id = self.world_frame
            m.ns = "arena"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            z = self.marker_height / 2.0
            for x, y in ((xn, yn), (xp, yn), (xp, yp), (xn, yp), (xn, yn)):
                m.points.append(Point(x=float(x), y=float(y), z=float(z)))
            out.append(m)
        return out

    def _publish_markers(self):
        """障害物本体（赤）と安全境界 R_total=obs_r+robot_r（薄ピンク）を RViz に出す。

        壁は巨大円なので円柱では描かず、_arena_markers() が箱の線として描く。
        """
        arr = MarkerArray()
        if not self.obstacles:
            # 障害物が無い（mocap 受信前など）→ DELETEALL で既存マーカーを消す
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            arr.markers.extend(self._arena_markers())  # 会場の箱は出しておく
            self.marker_pub.publish(arr)
            return
        h = self.marker_height
        for i, obs in enumerate(self.obstacles):
            ox, oy = float(obs.position[0]), float(obs.position[1])
            # 本体（物理半径）
            m = Marker()
            m.header.frame_id = self.world_frame
            m.ns = "obstacle"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = ox, oy, h / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * obs.radius
            m.scale.z = h
            m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.1, 0.1, 0.85
            arr.markers.append(m)
            # 安全境界（R_total = obs.radius + robot_radius）
            mm = Marker()
            mm.header.frame_id = self.world_frame
            mm.ns = "obstacle_margin"
            mm.id = i
            mm.type = Marker.CYLINDER
            mm.action = Marker.ADD
            mm.pose.position.x, mm.pose.position.y, mm.pose.position.z = ox, oy, h / 2.0 * 0.99
            mm.pose.orientation.w = 1.0
            mm.scale.x = mm.scale.y = 2.0 * (obs.radius + self.robot_radius)
            mm.scale.z = h * 0.98
            mm.color.r, mm.color.g, mm.color.b, mm.color.a = 1.0, 0.4, 0.7, 0.2
            arr.markers.append(mm)
        arr.markers.extend(self._arena_markers())
        self.marker_pub.publish(arr)

    # ---- body正規化指令 → world 速度[m/s]（収集と同じ規約）----
    def _body_to_world(self, lin_x, lin_y):
        bvx, bvy = lin_x * self.max_vxy, lin_y * self.max_vxy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c * bvx - s * bvy, s * bvx + c * bvy])

    # ---- world 速度[m/s] → body正規化指令 ----
    def _world_to_body(self, ux, uy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bvx = c * ux + s * uy
        bvy = -s * ux + c * uy
        return (float(np.clip(bvx / self.max_vxy, -1.0, 1.0)),
                float(np.clip(bvy / self.max_vxy, -1.0, 1.0)))

    def _build_obstacles(self, flat):
        """フラット配列 [x1,y1,r1, x2,y2,r2, ...] を Obstacle のリストにする。"""
        flat = list(flat)
        if len(flat) % 3 != 0:
            raise ValueError(f"obstacles は [x,y,r] の倍数で渡す: {flat}")
        return [
            self._Obstacle(position=np.array([flat[i], flat[i + 1]], dtype=float),
                           radius=float(flat[i + 2]))
            for i in range(0, len(flat), 3)
        ]

    def _on_obstacles(self, msg):
        """mocap 由来の障害物位置で self.obstacles を更新（位置が動いたらマーカーも再送）。"""
        if not msg.data:
            return  # 空配列は無視（まだ1個も見えていない等）。直前の障害物を保持
        try:
            self.obstacles = self._build_obstacles(msg.data)
        except ValueError as e:
            self.get_logger().warn(f"障害物メッセージが不正: {e}")
            return
        self._publish_markers()

    def _on_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        self.pos = np.array([p.x, p.y])
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

    def _on_cmd_nom(self, msg: Twist):
        self.u_nom_body = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z])

    def _tick(self):
        if not self.have_pose:
            return  # pose 未受信：何も出さない（暴走防止）

        # 生指令(body) → world 速度（xy）。z/旋回は後で素通し用に保持
        u_nom_world = self._body_to_world(self.u_nom_body[0], self.u_nom_body[1])

        # mocap 由来の障害物 + 会場境界の壁。壁は wall_enable=false なら空リスト。
        obs_all = self._all_obstacles()

        if self.mode in ("shield", "shield_multi"):
            # CVAE 文脈（直近 context_len の (x, u_nom_world)）を渡す。トラッキング補正あり。
            ctx_xs = np.array(self.ctx_x, dtype=np.float32)
            ctx_us = np.array(self.ctx_u, dtype=np.float32)
            u_safe_world = self.filter.filter(
                self.pos, u_nom_world, obs_all, self.robot_radius, ctx_xs, ctx_us
            )
        else:
            # 素のCBF：文脈なし・補正なし（d=0）。無入力なら u_safe=0。
            u_safe_world = self.filter.filter(
                self.pos, u_nom_world, obs_all, self.robot_radius
            )

        # 実際に使われたスラック[m/s]。>0 は「このステップは CBF の減衰条件を満たせず
        # 緩めた（＝安全余裕を削って進んだ）」の意味。slack 無効時も clip 違反を拾う。
        slack = float(getattr(self.filter, "last_slack", 0.0))
        if slack > 1e-9:
            self._slack_steps += 1
            self._slack_peak = max(self._slack_peak, slack)
            self.get_logger().warn(
                f"スラック作動 s={slack:.3f} m/s（累計 {self._slack_steps} steps, "
                f"ピーク {self._slack_peak:.3f}）", throttle_duration_sec=2.0)

        # world → body 正規化して publish
        bx, by = self._world_to_body(u_safe_world[0], u_safe_world[1])
        out = Twist()
        out.linear.x, out.linear.y = bx, by
        out.linear.z = float(self.u_nom_body[2]) if self.pass_z else 0.0
        out.angular.z = float(self.u_nom_body[3]) if self.pass_yaw else 0.0
        self.cmd_pub.publish(out)

        # 文脈更新：★フィルタ前の生指令(world)を保存（CVAEはそれで学習済）
        self.ctx_x.append(self.pos.copy())
        self.ctx_u.append(u_nom_world.copy())

        # --- バリア値 h・クリアランス sdf を最近傍障害物について計算 ---
        h, sdf = self._barrier_state(self.pos)

        # --- ライブ Path（RViz）---
        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(self.pos[0]), float(self.pos[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self._path.poses.append(ps)
        if len(self._path.poses) > 5000:
            self._path.poses = self._path.poses[-5000:]
        self._path.header.stamp = ps.header.stamp
        self.path_pub.publish(self._path)

        # --- 記録 ---
        if self.record:
            r = self._rec
            r["t"].append(time.perf_counter() - self._t0)
            r["px"].append(float(self.pos[0])); r["py"].append(float(self.pos[1]))
            r["yaw"].append(float(self.yaw))
            r["unom"].append(u_nom_world.copy())
            r["usafe"].append(np.array(u_safe_world, dtype=float))
            r["h"].append(h); r["sdf"].append(sdf)
            r["alpha"].append(float(getattr(self.filter, "_alpha", np.nan)))
            r["slack"].append(slack)

    def _barrier_state(self, pos):
        """最近傍障害物について h_smooth と sdf を返す（記録・可視化用。壁も含む）。"""
        obs_all = self._all_obstacles()
        if not obs_all:
            return float("nan"), float("inf")
        b = self.filter.barrier
        best_sdf, best_h = float("inf"), float("nan")
        for obs in obs_all:
            R = obs.radius + self.robot_radius
            s = b.sdf(pos, obs.position, R)
            if s < best_sdf:
                best_sdf = s
                best_h = b.h_smooth(pos, obs.position, R)
        return float(best_h), float(best_sdf)

    def save_record(self):
        """記録を log/traj/<日付>/<時刻>_<mode>.npz に保存。"""
        if not self.record or not self._rec["t"]:
            return
        now = datetime.now()
        out_dir = os.path.join(self.log_root, "traj", now.strftime("%Y-%m-%d"))
        os.makedirs(out_dir, exist_ok=True)
        # 機体名(ノード名)をファイル名に入れる：マルチエージェントで2機が同じ秒に
        # 終了しても上書きしない（既定ノード名 shield_filter_node のときは付けない）。
        node_name = self.get_name()
        tag = "" if node_name in ("shield_filter_node", "") else f"_{node_name}"
        out = os.path.join(out_dir, f"{now.strftime('%H%M%S')}{tag}_{self.mode}.npz")
        r = self._rec
        np.savez(
            out,
            t=np.array(r["t"]),
            x=np.column_stack([r["px"], r["py"]]),
            yaw=np.array(r["yaw"]),
            u_nom=np.array(r["unom"]),
            u_safe=np.array(r["usafe"]),
            h=np.array(r["h"]),
            sdf=np.array(r["sdf"]),
            alpha=np.array(r["alpha"]),
            slack=np.array(r["slack"]),
            obstacles=np.array([[o.position[0], o.position[1], o.radius]
                                for o in self.obstacles]),
            wall_bounds=np.array(self.wall_bounds if self.wall_enable else []),
            slack_weight=(np.nan if self.slack_weight is None else self.slack_weight),
            robot_radius=float(self.robot_radius),
            mode=self.mode,
        )
        n_unsafe = int(np.sum(np.array(r["sdf"]) < 0))
        sl = np.array(r["slack"])
        self.get_logger().info(
            f"記録保存: {out}  (N={len(r['t'])} steps, sdf<0 が {n_unsafe} steps, "
            f"slack>0 が {int(np.sum(sl > 1e-9))} steps / ピーク "
            f"{(float(np.max(sl)) if sl.size else 0.0):.3f} m/s)"
        )
        return out


def main():
    rclpy.init()
    node = ShieldFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時に停止指令＆記録の保存
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.save_record()
        except Exception as e:
            print(f"[shield] 記録保存に失敗: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
