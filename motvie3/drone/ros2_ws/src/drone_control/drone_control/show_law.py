# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""ドローンショー（ルールベース）の目標軌道（ROS 非依存の純粋関数）。

各機の目標は時刻だけで決まる（機体同士は情報をやり取りしない）:

    p_i*(t) = c(t) + ρ R(φ(t) + θ_i) e_x + dz_i e_z
    ṗ_i*(t) = ċ(t) + ρ φ̇(t) J R(φ(t) + θ_i) e_x        （J: 90° 回転）

    c, ċ : ショーの中心とその速度（formation_commander の Leader。'e' キーで楕円軌道）
    φ, φ̇ : 隊形全体の回転角とその速度（commander の 'o' キーで回転の開始/停止）
    ρ, θ_i : 中心からの距離と、隊形の中での位置（3 機で 120° ずつなら正三角形）

追従は consensus_law.compute_command に「相手 = 目標点 1 つ」を渡して計算する:
    u_i = ṗ_i* − k (p_i − p_i*)      … 式 (3.5) で近傍が目標点だけの場合と同じ（誤差は e^{−kt} で減る）

座標は ROS 規約（x=前, y=左, z=上, world）。
"""

import math
from typing import Tuple

from drone_control.swarm_config import DroneShowSpec, ShowSlotSpec

Vec3 = Tuple[float, float, float]


def slot_target(slot: ShowSlotSpec, radius: float, center: Vec3, center_vel: Vec3,
                phase: float, phase_rate: float) -> Tuple[Vec3, Vec3]:
    """(p_i*, ṗ_i*) を返す。"""
    ang = phase + slot.azimuth
    c, s = math.cos(ang), math.sin(ang)
    p = (center[0] + radius * c, center[1] + radius * s, center[2] + slot.dz)
    v = (center_vel[0] - radius * phase_rate * s, center_vel[1] + radius * phase_rate * c, center_vel[2])
    return p, v


def all_targets(spec: DroneShowSpec, center: Vec3, center_vel: Vec3, phase: float, phase_rate: float):
    return {n: slot_target(a, spec.radius, center, center_vel, phase, phase_rate) for n, a in spec.agents.items()}
