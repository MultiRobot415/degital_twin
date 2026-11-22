# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""ドローンショー（show_law.py / drone_show.yaml）のテスト。"""

import copy
import itertools
import math
import os

import pytest
import yaml

from drone_control import swarm_config
from drone_control.consensus_law import compute_command, leader_node
from drone_control.leader_trajectory import Ellipse
from drone_control.show_law import all_targets, slot_target

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SWARM = swarm_config.load_swarm(os.path.join(PKG, "config", "swarm.yaml"))
PATH = os.path.join(PKG, "config", "formation", "drone_show.yaml")
SPEC = swarm_config.load_drone_show(PATH, SWARM)
SIDE = SPEC.radius * math.sqrt(3.0)


def test_shipped_config():
    assert swarm_config.formation_law(PATH) == swarm_config.DRONE_SHOW
    assert SPEC.min_separation() == pytest.approx(SIDE)
    # 初期配置（tello2 右・tello3 左）と同じ側
    assert SPEC.agents["tello2"].azimuth == pytest.approx(math.radians(240))
    assert SPEC.agents["tello3"].azimuth == pytest.approx(math.radians(120))


@pytest.mark.parametrize("phase", [0.0, 0.7, 2.0, -2.9])
def test_equilateral_at_any_phase(phase):
    t = all_targets(SPEC, (0.3, -0.1, 1.0), (0.0, 0.0, 0.0), phase, 0.0)
    for a, b in itertools.combinations(t, 2):
        assert math.dist(t[a][0], t[b][0]) == pytest.approx(SIDE)
    cx = sum(p[0][0] for p in t.values()) / 3
    cy = sum(p[0][1] for p in t.values()) / 3
    assert (cx, cy) == pytest.approx((0.3, -0.1))      # 重心 = 中心


def test_velocity_is_time_derivative():
    """中心が楕円・隊形が回転しているとき、ṗ_i* が p_i*(t) の数値微分と一致する。"""
    ell = Ellipse((0.5, 0.0, 1.0), 0.0, SWARM.ellipse_a, SWARM.ellipse_b, SWARM.ellipse_period)
    w = SWARM.show_spin_rate
    slot = SPEC.agents["tello2"]

    def at(t):
        st = ell.state(t, 0.0, 0.02)
        return slot_target(slot, SPEC.radius, st.position, st.velocity, 0.3 + w * t, w)

    h = 1e-5
    for t in (1.0, 9.0, 23.0):
        num = [(a - b) / (2 * h) for a, b in zip(at(t + h)[0], at(t - h)[0])]
        assert at(t)[1] == pytest.approx(num, abs=1e-6)


def run_show(t_formation=8.0, t_end=70.0, dt=0.005):
    """swarm_sim と同じ運動モデル（正規化 → 速度、1 次遅れ τ=0.15 s）で閉ループを回す。
    0〜t_formation: 中心は静止・回転なしで三角形を組む → 以後: 回転 + 楕円周回。"""
    tau, vs_xy, vs_z = 0.15, SWARM.velocity_scale_xy, SWARM.velocity_scale_z
    r0 = SWARM.reference_initial
    pos = {d.name: [d.sim_initial_position[0], d.sim_initial_position[1], 0.8] for d in SWARM.drones}
    vel = {n: [0.0, 0.0, 0.0] for n in pos}
    cmd = {n: (0.0, 0.0, 0.0, 0.0) for n in pos}
    ell = Ellipse(r0, 0.0, SWARM.ellipse_a, SWARM.ellipse_b, SWARM.ellipse_period)
    w = SWARM.show_spin_rate
    period = int(round(1.0 / SPEC.rate_hz / dt))
    log, step, t = [], 0, 0.0
    while t <= t_end:
        if t < t_formation:
            center, cvel, phase, rate = r0, (0.0, 0.0, 0.0), 0.0, 0.0
        else:
            st = ell.state(t - t_formation, 0.0, 0.02)
            center, cvel, phase, rate = st.position, st.velocity, w * (t - t_formation), w
        targets = all_targets(SPEC, center, cvel, phase, rate)
        if step % period == 0:
            for n in pos:
                p_star, v_star = targets[n]
                cmd[n] = compute_command(tuple(pos[n]), 0.0, (0, 0, 0), (0, 0, 0), SPEC,
                                         [leader_node(p_star, v_star)], vs_xy, vs_z).body_cmd
        log.append((t, {n: tuple(pos[n]) for n in pos}, {n: targets[n][0] for n in pos}))
        for n in pos:
            v_cmd = (cmd[n][0] * vs_xy, cmd[n][1] * vs_xy, cmd[n][2] * vs_z)
            a = 1.0 - math.exp(-dt / tau)
            vel[n] = [v + (vc - v) * a for v, vc in zip(vel[n], v_cmd)]
            for ax in range(3):
                pos[n][ax] += vel[n][ax] * dt
        step, t = step + 1, t + dt
    return log


def test_show_closed_loop():
    log = run_show()
    # 回転 + 楕円の後半: 目標からの誤差が小さく、正三角形が保たれている
    tail = [entry for entry in log if entry[0] > 40.0]
    worst_err = max(math.dist(p[n], tg[n]) for _, p, tg in tail for n in p)
    worst_side = max(abs(math.dist(p[a], p[b]) - SIDE) for _, p, _ in tail
                     for a, b in itertools.combinations(p, 2))
    assert worst_err < 0.03, worst_err          # 1 次遅れ τ=0.15s ぶんの遅れ（≈ 速さ × τ）以内
    assert worst_side < 0.03, worst_side
    # 初期位置から三角形を組む途中も含め、機体同士が近づきすぎない
    d_min = min(math.dist(p[a], p[b]) for _, p, _ in log for a, b in itertools.combinations(p, 2))
    assert d_min > 0.55, d_min


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d.pop("law"), "law: drone_show"),
    (lambda d: d["agents"]["tello3"].update(azimuth_deg=240), "同じ位置"),
    (lambda d: d["agents"].pop("tello3"), "不足"),
    (lambda d: d["shape"].update(radius=0), "> 0.0"),
])
def test_invalid(mutate, message):
    with open(PATH, encoding="utf-8") as f:
        data = copy.deepcopy(yaml.safe_load(f))
    mutate(data)
    with pytest.raises(ValueError, match=message):
        swarm_config.parse_drone_show(data, SWARM)
