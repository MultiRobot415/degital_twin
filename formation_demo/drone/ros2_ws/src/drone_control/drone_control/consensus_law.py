# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""論文 第3章（1次システムのフォーメーション）の合意制御則（ROS 非依存の純粋関数）。

    u_i = (1 / Σ_j a_ij) Σ_j a_ij [ −k (r̂_i − r̂_j) + ṙ_j + (ḋ_i − ḋ_j) ]      … 式 (3.5)
    r̂_i = r_i − d_i                                                           … 式 (3.7)

    j は近傍（a_ij = 1）と Leader（a_i,N+1 = 1 のとき。d_{N+1} = 0）
    u_i : 機体 i の速度指令（world, m/s）, ṙ_j : 近傍 j の速度, d_i : Leader から見た目標相対位置

性質（定理3.1・系3.1）: すべての機体が近傍をたどって Leader の情報を得られるなら（仮定3.1）、
各機は r_L + d_i に収束し、誤差は e^{−kt} で減衰する。収束の速さはネットワーク構造に依存しない。

隊形（式 3.22）: d_i = |d_i| R(ψ + θ_i) e_x（ψ: Leader の進行方向, θ_i: 方位）。
rotate_with_leader=false のときは ψ=0 に固定（世界座標で向きが変わらない隊形）。
上下方向は論文の範囲外だが、同じ式を z にも適用する（d_z = dz, ḋ_z = 0）。

座標は ROS 規約（x=前, y=左, z=上, world）。出力の機体座標・正規化への変換は frames.py の規約に従う。
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from drone_control.frames import wrap_angle
from drone_control.swarm_config import ConsensusAgentSpec, ConsensusSpec

Vec3 = Tuple[float, float, float]
ZERO: Vec3 = (0.0, 0.0, 0.0)


def desired_offset(agent: ConsensusAgentSpec, heading: float, rotate: bool) -> Vec3:
    """d_i（式 3.22）。"""
    ang = (heading if rotate else 0.0) + agent.azimuth
    return (agent.distance * math.cos(ang), agent.distance * math.sin(ang), agent.dz)


def desired_offset_rate(agent: ConsensusAgentSpec, heading: float, heading_rate: float,
                        rotate: bool) -> Vec3:
    """ḋ_i = ψ̇ · ∂d_i/∂ψ（隊形が回るときだけ非ゼロ）。"""
    if not rotate:
        return ZERO
    ang = heading + agent.azimuth
    return (-agent.distance * math.sin(ang) * heading_rate,
            agent.distance * math.cos(ang) * heading_rate, 0.0)


@dataclass(frozen=True)
class NodeState:
    """制御則が使う「相手 j」の情報。Leader は offset = offset_rate = 0。"""
    position: Vec3       # r_j
    velocity: Vec3       # ṙ_j（近傍は「その機体が出している速度指令」を使う）
    offset: Vec3         # d_j
    offset_rate: Vec3    # ḋ_j


@dataclass(frozen=True)
class ConsensusCommand:
    world_velocity: Vec3                         # 飽和後の u_i（world, m/s）。近傍に ṙ_i として共有する
    body_cmd: Tuple[float, float, float, float]  # 正規化 cmd_vel（機体座標）
    target: Vec3                                 # 機体 i が推定する自分の目標位置 = mean_j(r_j − d_j) + d_i


def consensus_velocity(p_i: Vec3, d_i: Vec3, dd_i: Vec3, k: float,
                       others: Sequence[NodeState]) -> Tuple[Vec3, Vec3]:
    """式 (3.5) そのもの。(u_i, target_i) を返す。others は a_ij = 1 の相手（Leader を含む）。"""
    if not others:
        raise ValueError("Σa_ij = 0（Leader とも近傍ともつながっていない）")
    n = float(len(others))
    u = [0.0, 0.0, 0.0]
    anchor = [0.0, 0.0, 0.0]
    for o in others:
        for ax in range(3):
            r_hat_i = p_i[ax] - d_i[ax]
            r_hat_j = o.position[ax] - o.offset[ax]
            u[ax] += -k * (r_hat_i - r_hat_j) + o.velocity[ax] + (dd_i[ax] - o.offset_rate[ax])
            anchor[ax] += r_hat_j
    u = tuple(v / n for v in u)
    target = tuple(anchor[ax] / n + d_i[ax] for ax in range(3))
    return u, target


def compute_command(p_i: Vec3, yaw_i: float, d_i: Vec3, dd_i: Vec3, spec: ConsensusSpec,
                    others: Sequence[NodeState], velocity_scale_xy: float,
                    velocity_scale_z: float) -> ConsensusCommand:
    u, target = consensus_velocity(p_i, d_i, dd_i, spec.k, others)

    # 飽和（水平は方向を保ってノルムで、上下は個別に）
    ux, uy, uz = u
    norm = math.hypot(ux, uy)
    if norm > spec.max_speed_xy:
        s = spec.max_speed_xy / norm
        ux, uy = ux * s, uy * s
    uz = max(-spec.max_speed_z, min(spec.max_speed_z, uz))

    # m/s → 正規化指令、world → 機体座標（現在ヨーの逆回転）
    c, s = math.cos(yaw_i), math.sin(yaw_i)
    bx = (c * ux + s * uy) / velocity_scale_xy
    by = (-s * ux + c * uy) / velocity_scale_xy
    bz = uz / velocity_scale_z
    wz = spec.k_yaw * wrap_angle(spec.goal_yaw - yaw_i)
    wz = max(-spec.max_cmd_yaw, min(spec.max_cmd_yaw, wz))
    return ConsensusCommand(world_velocity=(ux, uy, uz), body_cmd=(bx, by, bz, wz), target=target)


def leader_node(position: Vec3, velocity: Vec3) -> NodeState:
    return NodeState(position, velocity, ZERO, ZERO)


def offset_pair(agent: ConsensusAgentSpec, heading: Optional[float], heading_rate: float,
                rotate: bool) -> Tuple[Vec3, Vec3]:
    h = heading if heading is not None else 0.0
    return desired_offset(agent, h, rotate), desired_offset_rate(agent, h, heading_rate, rotate)
