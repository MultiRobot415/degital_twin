# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""変位ベースのフォーメーション制御則（ROS 非依存の純粋関数）。

機体 i が使ってよい情報は「自分の位置 p_i・ヨー」「近傍 j∈N_i の位置 p_j」
「（pinned のときだけ）基準 r」だけ。これを軸ごとに次の式で指令にする:

    u_i = k_ref · a_i · (r + d_i − p_i)                               … 基準追従項
        + k_nbr · Σ_{j∈N_i} w_ij · ((p_j − d_j) − (p_i − d_i))        … 近傍合意項

    d_i : 隊形オフセット（基準からの変位）, a_i ∈ {0,1} : pinned か, w_ij > 0 : 重み

設定だけで次の制御則になる（formation/*.yaml）:
    全機 pinned・近傍なし              → 基準追従（ref の Tモードと同じ）
    リーダーだけ pinned・他はリーダーが近傍 → リーダー・フォロワー
    リーダーだけ pinned・近傍を相互に張る   → 合意（コンセンサス）型

u_i は「世界座標の正規化速度指令」。これを
    K_i = k_ref·a_i + k_nbr·Σ w_ij,   target_i = p_i + u_i / K_i
と書くと u_i = K_i (target_i − p_i)。target_i は「機体 i がいま向かっている点」で、
(r + d_i) と各近傍が示す (p_j − d_j + d_i) の重み付き平均になる（可視化・ログ用に出力する）。

出力は機体座標（ROS 規約）の正規化 cmd_vel。符号・座標系は frames.py を参照。
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from drone_control.frames import wrap_angle
from drone_control.swarm_config import AgentSpec, ControlSpec

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class NeighborState:
    position: Vec3   # p_j（world, ROS 座標）
    offset: Vec3     # d_j
    weight: float    # w_ij


@dataclass(frozen=True)
class FormationCommand:
    body_cmd: Tuple[float, float, float, float]   # (linear.x, linear.y, linear.z, angular.z) 正規化・機体座標
    world_cmd: Vec3                               # 飽和後の世界座標の指令（ログ用）
    target: Vec3                                  # 機体 i が向かっている点（world）


def compute_command(p_i: Vec3, yaw_i: float, agent: AgentSpec, control: ControlSpec,
                    reference: Optional[Vec3],
                    neighbors: Sequence[NeighborState]) -> FormationCommand:
    """1機分の指令を計算する。pinned なら reference は必須（None は呼び出し側の誤り）。"""
    if agent.pinned and reference is None:
        raise ValueError(f"{agent.name}: pinned なのに基準が与えられていない")

    d_i = agent.offset
    gains = (control.k_ref.xy, control.k_ref.xy, control.k_ref.z)
    nbr_gains = (control.k_nbr.xy, control.k_nbr.xy, control.k_nbr.z)

    u = [0.0, 0.0, 0.0]
    k_total = [0.0, 0.0, 0.0]
    for ax in range(3):
        if agent.pinned:
            u[ax] += gains[ax] * (reference[ax] + d_i[ax] - p_i[ax])
            k_total[ax] += gains[ax]
        for n in neighbors:
            u[ax] += nbr_gains[ax] * n.weight * ((n.position[ax] - n.offset[ax]) - (p_i[ax] - d_i[ax]))
            k_total[ax] += nbr_gains[ax] * n.weight

    target = tuple(p_i[ax] + u[ax] / k_total[ax] if k_total[ax] > 0.0 else p_i[ax] for ax in range(3))

    # --- 飽和: 水平は方向を保ったままノルムで制限、上下・ヨーは個別にクリップ ---
    ux, uy, uz = u
    norm_xy = math.hypot(ux, uy)
    if norm_xy > control.max_cmd_xy:
        s = control.max_cmd_xy / norm_xy
        ux, uy = ux * s, uy * s
    uz = max(-control.max_cmd_z, min(control.max_cmd_z, uz))

    # --- 世界座標 → 機体座標（現在ヨーの逆回転）。回転はノルムを保つので飽和と順序は交換可能 ---
    c, s = math.cos(yaw_i), math.sin(yaw_i)
    bx = c * ux + s * uy
    by = -s * ux + c * uy

    # --- ヨー: 全機共通の目標ヨーへ最短角で ---
    wz = control.k_yaw * wrap_angle(control.goal_yaw - yaw_i)
    wz = max(-control.max_cmd_yaw, min(control.max_cmd_yaw, wz))

    return FormationCommand(body_cmd=(bx, by, uz, wz), world_cmd=(ux, uy, uz), target=target)
