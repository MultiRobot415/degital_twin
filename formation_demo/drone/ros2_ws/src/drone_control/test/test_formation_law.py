# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フォーメーション制御則（formation_law.py）のテスト。"""

import math
import os

import numpy as np
import pytest

from drone_control import swarm_config
from drone_control.formation_law import NeighborState, compute_command
from drone_control.swarm_config import AgentSpec, AxisGain, ControlSpec

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SWARM = swarm_config.load_swarm(os.path.join(PKG, "config", "swarm.yaml"))


def control(**kw):
    base = dict(rate_hz=20.0, k_ref=AxisGain(0.5, 0.6), k_nbr=AxisGain(0.4, 0.3), k_yaw=1.0,
                goal_yaw=0.0, max_cmd_xy=1.0, max_cmd_z=1.0, max_cmd_yaw=1.0)
    base.update(kw)
    return ControlSpec(**base)


def formation(name):
    return swarm_config.load_formation(os.path.join(PKG, "config", "formation", f"{name}.yaml"), SWARM)


def test_zero_command_at_formation_equilibrium():
    r = (0.3, -0.2, 1.1)
    d_i, d_j = (-0.4, 0.4, 0.0), (0.0, 0.0, 0.0)
    agent = AgentSpec("i", d_i, True, (("j", 1.0),))
    p_i = tuple(a + b for a, b in zip(r, d_i))
    p_j = tuple(a + b for a, b in zip(r, d_j))
    out = compute_command(p_i, 0.0, agent, control(), r, [NeighborState(p_j, d_j, 1.0)])
    assert all(abs(v) < 1e-12 for v in out.body_cmd)
    assert all(abs(a - b) < 1e-12 for a, b in zip(out.target, p_i))


def test_reference_term_matches_position_controller():
    """pinned・近傍なしなら既存 position_controller と同じ P 制御（世界誤差 × ゲイン）。"""
    agent = AgentSpec("i", (0.0, 0.0, 0.0), True, ())
    out = compute_command((0.0, 0.0, 1.0), 0.0, agent, control(), (0.4, -0.2, 1.5), [])
    assert out.body_cmd[:3] == pytest.approx((0.5 * 0.4, 0.5 * -0.2, 0.6 * 0.5))


def test_world_to_body_rotation():
    """機首が +y（yaw=90°）のとき、世界 +x への誤差は機体の右（body y < 0）。"""
    agent = AgentSpec("i", (0.0, 0.0, 0.0), True, ())
    ctl = control(goal_yaw=math.pi / 2)
    out = compute_command((0.0, 0.0, 1.0), math.pi / 2, agent, ctl, (1.0, 0.0, 1.0), [])
    bx, by, bz, wz = out.body_cmd
    assert abs(bx) < 1e-12 and by == pytest.approx(-0.5) and bz == 0.0 and abs(wz) < 1e-12


def test_yaw_takes_shortest_path():
    agent = AgentSpec("i", (0.0, 0.0, 0.0), True, ())
    ctl = control(goal_yaw=math.radians(170), max_cmd_yaw=10.0)
    out = compute_command((0, 0, 1), math.radians(-170), agent, ctl, (0, 0, 1), [])
    assert out.body_cmd[3] == pytest.approx(-math.radians(20))   # +340° ではなく -20°


def test_horizontal_saturation_keeps_direction():
    agent = AgentSpec("i", (0.0, 0.0, 0.0), True, ())
    out = compute_command((0, 0, 1), 0.0, agent, control(max_cmd_xy=0.5, max_cmd_z=0.2),
                          (3.0, 4.0, 9.0), [])
    bx, by, bz, _ = out.body_cmd
    assert math.hypot(bx, by) == pytest.approx(0.5)
    assert by / bx == pytest.approx(4.0 / 3.0)       # 方向（3:4）が保たれる
    assert bz == pytest.approx(0.2)


def test_target_is_weighted_average():
    """target = (k_ref·(r+d_i) + k_nbr·Σw(p_j−d_j+d_i)) / (k_ref + k_nbr·Σw)（軸ごと）。"""
    ctl = control(k_ref=AxisGain(0.5, 0.5), k_nbr=AxisGain(0.25, 0.25))
    d_i = (0.1, 0.2, 0.0)
    agent = AgentSpec("i", d_i, True, (("a", 1.0), ("b", 2.0)))
    r = (1.0, 0.0, 1.0)
    na = NeighborState((0.0, 1.0, 1.2), (0.0, 0.0, 0.0), 1.0)
    nb = NeighborState((2.0, -1.0, 0.8), (0.5, 0.0, 0.0), 2.0)
    out = compute_command((0.0, 0.0, 0.0), 0.0, agent, ctl, r, [na, nb])
    for ax in range(3):
        num = 0.5 * (r[ax] + d_i[ax])
        num += 0.25 * 1.0 * (na.position[ax] - na.offset[ax] + d_i[ax])
        num += 0.25 * 2.0 * (nb.position[ax] - nb.offset[ax] + d_i[ax])
        assert out.target[ax] == pytest.approx(num / (0.5 + 0.25 * 3.0))


def test_pinned_without_reference_is_an_error():
    agent = AgentSpec("i", (0.0, 0.0, 0.0), True, ())
    with pytest.raises(ValueError):
        compute_command((0, 0, 0), 0.0, agent, control(), None, [])


def slowest_rate(spec, axis):
    """隊形誤差 ξ̇ = −(k_nbr L + k_ref A) ξ の固有値実部の最小値（正規化指令 1 = 速度 1 のとき）。"""
    names = list(spec.agents)
    k_ref = getattr(spec.control.k_ref, axis)
    k_nbr = getattr(spec.control.k_nbr, axis)
    m = np.zeros((len(names), len(names)))
    for i, n in enumerate(names):
        a = spec.agents[n]
        m[i, i] += k_ref if a.pinned else 0.0
        for j_name, w in a.neighbors:
            j = names.index(j_name)
            m[i, i] += k_nbr * w
            m[i, j] -= k_nbr * w
    return float(np.min(np.linalg.eigvals(m).real))


@pytest.mark.parametrize("name, lam_xy", [
    ("reference_tracking", 0.5),       # 全機 k_ref のみ → λ = k_ref
    ("leader_follower", 0.5),          # 鎖状 → λ = min(k_ref, k_nbr)
    ("consensus", None),               # 1機だけ pinned → 全体モードが遅くなる（下で値を確認）
])
def test_slowest_mode(name, lam_xy):
    lam = slowest_rate(formation(name), "xy")
    assert lam > 0.0                    # well-posed（check_well_posed と整合）
    if lam_xy is not None:
        assert lam == pytest.approx(lam_xy)
    else:
        assert lam < 0.5                # 基準追従より遅い = 実験時間の見積もりに効く


@pytest.mark.parametrize("name", ["reference_tracking", "leader_follower", "consensus"])
def test_closed_loop_converges_to_formation(name):
    """swarm_sim と同じ運動モデル（正規化 → 速度、1次遅れ）で閉ループを回し、
    全機が r + d_i に収束することを確認する（同じ yaml・同じ制御則のコードを使う）。"""
    spec = formation(name)
    max_vxy, max_vz, tau, dt = 1.0, 0.6, 0.15, 0.01
    r = (0.8, -0.3, 1.2)
    pos = {d.name: [d.sim_initial_position[0], d.sim_initial_position[1], 0.8] for d in SWARM.drones}
    yaw = {n: 0.4 for n in pos}  # ヨーも 0 からずらして、機体座標変換を含めて検証
    vel = {n: [0.0, 0.0, 0.0] for n in pos}
    cmd = {n: (0.0, 0.0, 0.0, 0.0) for n in pos}
    period = int(round(1.0 / spec.control.rate_hz / dt))
    # 線形理論での最も遅いモードの減衰率 λ_min（1/s）から、誤差が e^-12 になる時間だけ回す
    lam = min(slowest_rate(spec, "xy") * max_vxy, slowest_rate(spec, "z") * max_vz)
    t_end = 12.0 / lam + 5.0
    for step in range(int(t_end / dt)):
        if step % period == 0:  # agent は rate_hz で指令を更新、Sim は保持
            for n, a in spec.agents.items():
                nbrs = [NeighborState(tuple(pos[j]), spec.agents[j].offset, w) for j, w in a.neighbors]
                cmd[n] = compute_command(tuple(pos[n]), yaw[n], a, spec.control,
                                         r if a.pinned else None, nbrs).body_cmd
        for n in pos:
            lx, ly, lz, az = cmd[n]
            v_cmd = (lx * max_vxy, ly * max_vxy, lz * max_vz)
            alpha = 1.0 - math.exp(-dt / tau)
            vel[n] = [v + (vc - v) * alpha for v, vc in zip(vel[n], v_cmd)]
            c, s = math.cos(yaw[n]), math.sin(yaw[n])
            pos[n][0] += (c * vel[n][0] - s * vel[n][1]) * dt
            pos[n][1] += (s * vel[n][0] + c * vel[n][1]) * dt
            pos[n][2] += vel[n][2] * dt
            yaw[n] += az * 3.0 * dt
    for n, a in spec.agents.items():
        goal = [r[ax] + a.offset[ax] for ax in range(3)]
        assert pos[n] == pytest.approx(goal, abs=5e-3), n
        assert abs(yaw[n]) < 1e-3
