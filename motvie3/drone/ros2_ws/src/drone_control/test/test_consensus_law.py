# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""論文 第3章の合意フォーメーション（consensus_law.py / leader_trajectory.py / 設定）のテスト。

論文の主張そのものを数値で確認する:
    定理3.1  各機は r_L + d_i に収束する
    系3.1    誤差は e^{−kt} で減衰し、その速さはネットワーク構造（NET1 / NET2）によらない
"""

import copy
import math
import os

import pytest
import yaml

from drone_control import swarm_config
from drone_control.consensus_law import (NodeState, compute_command, consensus_velocity, desired_offset,
                                         desired_offset_rate, leader_node, offset_pair)
from drone_control.leader_trajectory import Ellipse, LeaderState, heading_from_motion
from drone_control.swarm_config import ConsensusAgentSpec

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SWARM = swarm_config.load_swarm(os.path.join(PKG, "config", "swarm.yaml"))


def fpath(name):
    return os.path.join(PKG, "config", "formation", f"{name}.yaml")


def spec(name):
    return swarm_config.load_consensus(fpath(name), SWARM)


# ------------------------------------------------------------------ 隊形（式 3.22）
def test_desired_offset_geometry():
    a = ConsensusAgentSpec("i", 0.4, math.radians(90), 0.1, True, ())
    assert desired_offset(a, 0.0, True) == pytest.approx((0.0, 0.4, 0.1))          # 進行方向 +x の左
    assert desired_offset(a, math.pi / 2, True) == pytest.approx((-0.4, 0.0, 0.1))  # 進行方向 +y の左 = -x
    assert desired_offset(a, math.pi / 2, False) == pytest.approx((0.0, 0.4, 0.1))  # 回さない設定


def test_offset_rate_is_time_derivative():
    a = ConsensusAgentSpec("i", 0.4, math.radians(30), 0.0, True, ())
    psi, rate, h = 0.7, 0.3, 1e-6
    num = [(p - m) / (2 * h) for p, m in zip(desired_offset(a, psi + rate * h, True),
                                             desired_offset(a, psi - rate * h, True))]
    assert desired_offset_rate(a, psi, rate, True) == pytest.approx(num, abs=1e-8)
    assert desired_offset_rate(a, psi, rate, False) == (0.0, 0.0, 0.0)


# ------------------------------------------------------------------ 制御則（式 3.5）
def test_equilibrium_moves_with_leader():
    """全機が r_L + d_i にいて Leader と同じ速度なら、u_i = ṙ_L（隊形を保ったまま一緒に動く）。"""
    r_l, v_l = (0.3, -0.2, 1.0), (0.05, 0.02, 0.0)
    d_i, d_j = (0.4, 0.0, 0.0), (0.0, 0.4, 0.0)
    p_i = tuple(a + b for a, b in zip(r_l, d_i))
    p_j = tuple(a + b for a, b in zip(r_l, d_j))
    u, target = consensus_velocity(p_i, d_i, (0, 0, 0), 1.0,
                                   [NodeState(p_j, v_l, d_j, (0, 0, 0)), leader_node(r_l, v_l)])
    assert u == pytest.approx(v_l)
    assert target == pytest.approx(p_i)


def test_normalized_by_degree():
    """1/Σa_ij で正規化される: 同じ誤差の相手が 1 機でも 2 機でも指令は同じ。"""
    one = consensus_velocity((1, 0, 0), (0, 0, 0), (0, 0, 0), 2.0, [leader_node((0, 0, 0), (0, 0, 0))])[0]
    zero = (0.0, 0.0, 0.0)
    two = consensus_velocity((1, 0, 0), zero, zero, 2.0,
                             [leader_node(zero, zero), NodeState(zero, zero, zero, zero)])[0]
    assert one == pytest.approx((-2.0, 0.0, 0.0)) and two == pytest.approx(one)


def test_no_neighbors_is_error():
    with pytest.raises(ValueError):
        consensus_velocity((0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, [])


def test_command_conversion_and_saturation():
    s = spec("ch3_net2")
    # 速度 (0.6, 0, 0) m/s → 上限 0.3 でノルム飽和 → 正規化（scale 1.0）→ yaw 90° なら機体の右 = body -y
    others = [leader_node((0.6 / s.k, 0.0, 1.0), (0.0, 0.0, 0.0))]
    out = compute_command((0.0, 0.0, 1.0), math.pi / 2, (0, 0, 0), (0, 0, 0), s, others, 1.0, 0.6)
    assert out.world_velocity == pytest.approx((0.3, 0.0, 0.0))
    assert out.body_cmd[0] == pytest.approx(0.0, abs=1e-12) and out.body_cmd[1] == pytest.approx(-0.3)
    # 上下は velocity_scale_z で割る
    others = [leader_node((0.0, 0.0, 1.1), (0.0, 0.0, 0.0))]
    out = compute_command((0.0, 0.0, 1.0), 0.0, (0, 0, 0), (0, 0, 0), s, others, 1.0, 0.5)
    assert out.body_cmd[2] == pytest.approx(s.k * 0.1 / 0.5)


# ------------------------------------------------------------------ 閉ループ（定理3.1・系3.1）
def simulate(name, leader_fn, t_end, dt=0.002, start=None):
    """単積分器 ṙ_i = u_i（論文の式 3.1）で閉ループを回す。ṙ_j には近傍の 1 ステップ前の指令を使う
    （実装と同じ: 近傍は /formation_velocity で自分の指令を共有する）。飽和は外す。"""
    s = spec(name)
    s = swarm_config.ConsensusSpec(**{**s.__dict__, "max_speed_xy": 1e9, "max_speed_z": 1e9})
    pos = {d.name: list(start[d.name] if start else (d.sim_initial_position[0], d.sim_initial_position[1], 0.8))
           for d in SWARM.drones}
    vel = {n: (0.0, 0.0, 0.0) for n in pos}
    log = []
    t = 0.0
    while t <= t_end + 1e-12:
        L: LeaderState = leader_fn(t)
        errs, new_vel = {}, {}
        for n, a in s.agents.items():
            d_i, dd_i = offset_pair(a, L.heading, L.heading_rate, s.rotate_with_leader)
            others = []
            for j in a.neighbors:
                d_j, dd_j = offset_pair(s.agents[j], L.heading, L.heading_rate, s.rotate_with_leader)
                others.append(NodeState(tuple(pos[j]), vel[j], d_j, dd_j))
            if a.leader:
                others.append(leader_node(L.position, L.velocity))
            u, _ = consensus_velocity(tuple(pos[n]), d_i, dd_i, s.k, others)
            new_vel[n] = u
            errs[n] = [pos[n][ax] - (L.position[ax] + d_i[ax]) for ax in range(3)]
        log.append((t, errs))
        for n in pos:
            for ax in range(3):
                pos[n][ax] += new_vel[n][ax] * dt
        vel = new_vel
        t += dt
    return s, log


def static_leader(t):
    return LeaderState((0.5, 0.0, 1.0), (0.0, 0.0, 0.0), 0.0, 0.0)


@pytest.mark.parametrize("name", ["ch3_net1", "ch3_net2"])
def test_error_decays_as_exp_minus_kt(name):
    """系3.1: 静止 Leader のとき e_i(t) = e_i(0)·e^{−kt}（全機・全軸）。"""
    s, log = simulate(name, static_leader, t_end=4.0)
    e0 = log[0][1]
    for t, errs in log[::250]:
        for n in errs:
            for ax in range(3):
                assert errs[n][ax] == pytest.approx(e0[n][ax] * math.exp(-s.k * t), abs=0.01), (name, n, ax, t)


def test_convergence_speed_independent_of_network():
    """系3.1: NET1 と NET2 で誤差の時間変化がほぼ一致する（論文 Case I と II）。"""
    _, a = simulate("ch3_net1", static_leader, t_end=3.0)
    _, b = simulate("ch3_net2", static_leader, t_end=3.0)
    for (t, ea), (_, eb) in zip(a[::100], b[::100]):
        for n in ea:
            for ax in range(3):
                assert ea[n][ax] == pytest.approx(eb[n][ax], abs=0.01), (n, ax, t)


@pytest.mark.parametrize("name", ["ch3_net1", "ch3_net2"])
def test_tracks_ellipse_with_rotating_formation(name):
    """定理3.1: Leader が楕円を動き、隊形が進行方向に合わせて回っても、全機が r_L(t) + d_i(t) に収束する。"""
    ell = Ellipse((0.5, 0.0, 1.0), 0.0, SWARM.ellipse_a, SWARM.ellipse_b, SWARM.ellipse_period)
    state = {"h": 0.0}

    def leader(t):
        st = ell.state(t, state["h"], SWARM.heading_speed_threshold)
        state["h"] = st.heading
        return st

    _, log = simulate(name, leader, t_end=20.0)
    t, errs = log[-1]
    for n, e in errs.items():
        assert math.sqrt(sum(v * v for v in e)) < 5e-3, (name, n, e)


# ------------------------------------------------------------------ Leader の軌道
def test_ellipse_starts_at_current_point_and_heading():
    for heading in (0.0, 0.8, -2.0):
        ell = Ellipse((0.5, 0.0, 1.0), heading, 0.6, 0.5, 40.0)
        st = ell.state(0.0, heading, 0.02)
        assert st.position == pytest.approx((0.5, 0.0, 1.0))
        assert st.heading == pytest.approx(heading)


def test_heading_rate_matches_numeric_derivative():
    ell = Ellipse((0.0, 0.0, 1.0), 0.0, 0.6, 0.5, 40.0)
    h = 1e-4
    for t in (1.0, 7.0, 13.0, 31.0):
        a, b = ell.state(t - h, 0.0, 0.02).heading, ell.state(t + h, 0.0, 0.02).heading
        num = math.atan2(math.sin(b - a), math.cos(b - a)) / (2 * h)
        assert ell.state(t, 0.0, 0.02).heading_rate == pytest.approx(num, rel=1e-4)


def test_heading_held_when_slow():
    assert heading_from_motion(0.001, 0.0, 0.0, 0.0, 1.2, 0.02) == (1.2, 0.0)


# ------------------------------------------------------------------ 設定
@pytest.mark.parametrize("name", ["ch3_net1", "ch3_net2"])
def test_shipped_files(name):
    s = spec(name)
    assert swarm_config.formation_law(fpath(name)) == swarm_config.CONSENSUS_CH3
    assert s.agents["tello1"].leader and s.rotate_with_leader
    # 初期配置（tello2 右・tello3 左）と同じ側に目標を置き、経路が交差しないこと
    assert s.agents["tello2"].azimuth == pytest.approx(math.radians(270))
    assert s.agents["tello3"].azimuth == pytest.approx(math.radians(90))


def test_old_formations_are_not_consensus():
    assert swarm_config.formation_law(fpath("leader_follower")) == "displacement"


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d.pop("law"), "law: consensus_ch3"),
    (lambda d: d["agents"]["tello3"].update(neighbors=[]), "つながっていない"),
    # tello2 ⇄ tello3 だけで Leader につながらない
    (lambda d: (d["agents"]["tello1"].update(neighbors=[]), d["agents"]["tello2"].update(neighbors=["tello3"])),
     "Leader に到達できない"),
    (lambda d: d["agents"]["tello2"].update(neighbors=["tello9"]), "存在しない"),
    (lambda d: d["agents"]["tello2"].update(neighbors=["tello2"]), "自分自身"),
    (lambda d: d["agents"]["tello2"].update(neighbors=["tello1", "tello1"]), "重複"),
    (lambda d: d["control"].update(k=0), "> 0.0"),
    (lambda d: d["agents"].pop("tello3"), "不足"),
])
def test_invalid_consensus(mutate, message):
    with open(fpath("ch3_net2"), encoding="utf-8") as f:
        data = copy.deepcopy(yaml.safe_load(f))
    mutate(data)
    with pytest.raises(ValueError, match=message):
        swarm_config.parse_consensus(data, SWARM)


# ------------------------------------------------------------------ ゴール指定（GoTo）
def test_goto_phases_and_end_state():
    from drone_control.leader_trajectory import GoTo
    g = GoTo((0.5, 0.0, 1.0), 0.0, (-0.5, 0.0, 1.2), speed=0.1, turn_rate=0.3, final_heading=math.pi / 2)
    # ① 後ろ向き → 180° 回る（3.14/0.3 ≈ 10.5 s）② 約 1.02 m 直進 ③ 進行方向 180° から 90° へ回る
    assert g.t1 == pytest.approx(math.pi / 0.3)
    assert g.duration == pytest.approx(math.pi / 0.3 + math.hypot(1.0, 0.2) / 0.1 + (math.pi / 2) / 0.3)
    end = g.state(g.duration + 1.0)
    assert end.position == pytest.approx((-0.5, 0.0, 1.2)) and end.velocity == (0.0, 0.0, 0.0)
    assert end.heading == pytest.approx(math.pi / 2) and end.heading_rate == 0.0


def test_goto_is_continuous_and_heading_rate_consistent():
    """位置・向きが跳ばない（隊形が跳ばない）。heading_rate は向きの時間微分と一致する。"""
    from drone_control.leader_trajectory import GoTo
    g = GoTo((0.5, 0.0, 1.0), 0.3, (-0.4, -0.6, 1.0), speed=0.1, turn_rate=0.3, final_heading=-2.5)
    dt, prev = 0.01, g.state(0.0)
    t = dt
    while t < g.duration + 0.5:
        st = g.state(t)
        dh = math.atan2(math.sin(st.heading - prev.heading), math.cos(st.heading - prev.heading))
        assert abs(dh) <= 0.3 * dt + 1e-9                                    # 回転速度の上限を守る
        assert math.dist(st.position, prev.position) <= 0.1 * dt + 1e-9      # 位置が跳ばない
        if abs(st.heading_rate) > 0 and abs(prev.heading_rate) > 0:
            assert dh / dt == pytest.approx(st.heading_rate, abs=1e-6)
        prev, t = st, t + dt


def test_height_only_goal_does_not_turn():
    from drone_control.leader_trajectory import GoTo
    g = GoTo((0.5, 0.0, 1.0), 1.0, (0.5, 0.0, 1.4), speed=0.1, turn_rate=0.3)
    assert g.t1 == 0.0 and g.state(1.0).heading == pytest.approx(1.0)


@pytest.mark.parametrize("name", ["ch3_net1", "ch3_net2"])
def test_goal_behind_does_not_cross_paths(name):
    """後ろ向きのゴールでも隊形は形を保ったまま回る: 機体間の距離が初期値（0.566 m）からほぼ縮まない。"""
    from drone_control.leader_trajectory import GoTo
    s = spec(name)
    r0 = (0.5, 0.0, 1.0)
    start = {n: tuple(r0[i] + desired_offset(a, 0.0, True)[i] for i in range(3)) for n, a in s.agents.items()}
    g = GoTo(r0, 0.0, (-0.5, 0.0, 1.0), speed=0.1, turn_rate=0.3, final_heading=0.0)
    s2, log = simulate(name, g.state, t_end=g.duration + 5.0, start=start)

    # simulate は誤差しか返さないので、誤差 + 目標 から位置を復元して機体間距離を見る
    def positions(t, errs):
        st = g.state(t)
        return {n: [st.position[i] + desired_offset(a, st.heading, True)[i] + errs[n][i] for i in range(3)]
                for n, a in s2.agents.items()}
    d_min = min(math.dist(p[a], p[b]) for t, e in log[::50] for p in [positions(t, e)]
                for a, b in (("tello1", "tello2"), ("tello1", "tello3"), ("tello2", "tello3")))
    assert d_min > 0.55, d_min
    t, errs = log[-1]
    assert max(math.sqrt(sum(v * v for v in e)) for e in errs.values()) < 5e-3


# ------------------------------------------------------------------ 楕円の中心を固定する（ellipse.center）
@pytest.mark.parametrize("heading", [0.0, 1.0, -2.2, math.pi])
def test_ellipse_start_point_gives_configured_center(heading):
    """開始点に Leader を置いて今の向きで楕円を作ると、中心が指定どおりになる（向きが何度でも）。"""
    from drone_control.leader_trajectory import ellipse_start_point
    center = (0.1, -0.2, 1.0)
    start = ellipse_start_point(center, heading, 0.45, 0.35)
    ell = Ellipse(start, heading, 0.45, 0.35, 40.0)
    assert ell.center == pytest.approx(center)
    assert ell.state(0.0, heading, 0.02).heading == pytest.approx(heading)   # 走り出しで向きが跳ばない


def test_goto_without_turn_keeps_heading():
    from drone_control.leader_trajectory import GoTo
    g = GoTo((0.5, 0.0, 1.0), 0.7, (0.0, -0.35, 1.0), speed=0.1, turn_rate=0.3, final_heading=0.0, turn=False)
    assert g.duration == pytest.approx(math.hypot(0.5, 0.35) / 0.1)
    for t in (0.0, 2.0, 5.0, g.duration + 1):
        st = g.state(t)
        assert st.heading == pytest.approx(0.7) and st.heading_rate == 0.0
    assert g.state(g.duration + 1).position == pytest.approx((0.0, -0.35, 1.0))


def test_shipped_ellipse_center():
    assert SWARM.ellipse_center == (0.0, 0.0)
