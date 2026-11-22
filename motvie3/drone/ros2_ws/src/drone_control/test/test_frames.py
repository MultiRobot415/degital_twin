# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""座標・符号変換（frames.py）のテスト。ここが間違うと全機が間違った方向へ飛ぶ。"""

import math
import random

import pytest

from drone_control.frames import (format_rc, motive_yup_to_ros, quat_from_yaw, ros_to_motive_yup,
                                  twist_to_tello_rc, wrap_angle, yaw_from_quat)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    """クォータニオン q (x,y,z,w) でベクトル v を回す。"""
    x, y, z, w = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def axis_angle(axis, angle):
    s = math.sin(angle / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def random_unit_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def approx(a, b, tol=1e-12):
    return all(abs(x - y) < tol for x, y in zip(a, b))


IDENTITY = (0.0, 0.0, 0.0, 1.0)


# ------------------------------------------------------------------ 位置の軸対応
@pytest.mark.parametrize("motive, ros, meaning", [
    ((1, 0, 0), (1, 0, 0), "前"),
    ((0, 1, 0), (0, 0, 1), "上"),
    ((0, 0, 1), (0, -1, 0), "右（Motive +z）→ ROS -y"),
])
def test_axes_motive_to_ros(motive, ros, meaning):
    p, _ = motive_yup_to_ros(motive, IDENTITY)
    assert approx(p, ros), meaning


def test_conversion_is_proper_rotation_about_x():
    """位置変換は X 軸まわり +90° の回転そのもの（右手系を保つ = 鏡映ではない）。"""
    c = axis_angle((1, 0, 0), math.pi / 2)
    rng = random.Random(0)
    for _ in range(100):
        v = tuple(rng.uniform(-3, 3) for _ in range(3))
        p, _ = motive_yup_to_ros(v, IDENTITY)
        assert approx(p, rotate(c, v))


def test_round_trip():
    rng = random.Random(1)
    for _ in range(200):
        p = tuple(rng.uniform(-5, 5) for _ in range(3))
        q = random_unit_quat(rng)
        p2, q2 = ros_to_motive_yup(*motive_yup_to_ros(p, q))
        assert approx(p, p2) and approx(q, q2)
        p3, q3 = motive_yup_to_ros(*ros_to_motive_yup(p, q))
        assert approx(p, p3) and approx(q, q3)


def test_orientation_is_conjugated_consistently():
    """Motive で姿勢 q_M が機体ベクトル v を回した結果を ROS に写すと、
    ROS 側で q_ROS が「写した v」を回した結果と一致する（R_ROS = C R_M C^T）。"""
    rng = random.Random(2)
    for _ in range(200):
        q_m = random_unit_quat(rng)
        v = tuple(rng.uniform(-1, 1) for _ in range(3))
        _, q_r = motive_yup_to_ros((0, 0, 0), q_m)
        lhs, _ = motive_yup_to_ros(rotate(q_m, v), IDENTITY)
        v_r, _ = motive_yup_to_ros(v, IDENTITY)
        assert approx(lhs, rotate(q_r, v_r), 1e-9)


@pytest.mark.parametrize("theta", [0.3, 1.0, -1.2, 2.9])
def test_yaw_in_motive_becomes_ros_yaw(theta):
    """Motive で上(+Y)まわりに θ 回った機体は、ROS で yaw = θ（左回り正）になる。
    どちらも機首が「左」に向く回転であることも確認する。"""
    q_m = axis_angle((0, 1, 0), theta)
    nose_m = rotate(q_m, (1, 0, 0))
    assert nose_m[2] * math.sin(theta) < 0 or abs(math.sin(theta)) < 1e-9  # Motive: -z 側 = 左へ
    _, q_r = motive_yup_to_ros((0, 0, 0), q_m)
    assert abs(wrap_angle(yaw_from_quat(*q_r) - theta)) < 1e-9
    nose_r = rotate(q_r, (1, 0, 0))
    assert nose_r[1] * math.sin(theta) > 0 or abs(math.sin(theta)) < 1e-9   # ROS: +y 側 = 左へ


# ------------------------------------------------------------------ Tello rc
def test_rc_axes_and_signs():
    assert twist_to_tello_rc(0.5, 0, 0, 0) == (0, 50, 0, 0)      # 前 → b+
    assert twist_to_tello_rc(0, 0.5, 0, 0) == (-50, 0, 0, 0)     # 左(ROS +y) → a-（Tello は右が正）
    assert twist_to_tello_rc(0, -0.5, 0, 0) == (50, 0, 0, 0)     # 右 → a+
    assert twist_to_tello_rc(0, 0, 0.5, 0) == (0, 0, 50, 0)      # 上 → c+
    assert twist_to_tello_rc(0, 0, 0, 0.3) == (0, 0, 0, -30)     # 反時計回り(ROS +) → d-（Tello は時計回りが正）


def test_rc_matches_tello_driver_formula():
    """tello_ros/tello_driver: rc = round(-100 y), round(100 x), round(100 z), round(-100 az)（C++ round）。"""
    def cpp_round(v):
        return int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))
    rng = random.Random(3)
    for _ in range(500):
        x, y, z, a = (rng.uniform(-1, 1) for _ in range(4))
        assert twist_to_tello_rc(x, y, z, a) == (cpp_round(-100 * y), cpp_round(100 * x),
                                                 cpp_round(100 * z), cpp_round(-100 * a))


def test_rc_rounding_and_clamp():
    assert twist_to_tello_rc(0.005, 0, 0, 0)[1] == 1       # 0.5 は 0 から遠い側へ（Python round なら 0）
    assert twist_to_tello_rc(-0.005, 0, 0, 0)[1] == -1
    assert twist_to_tello_rc(2.0, -3.0, 1.5, -9.0) == (100, 100, 100, 100)
    assert format_rc((1, -2, 3, -4)) == "rc 1 -2 3 -4"


def test_chain_consistent_with_ref_flight_log():
    """ref の実機ログ: rc_lr > 0 で mocap_z（Motive +z = 右）が増える。
    ROS で「右へ」(linear.y < 0) の指令 → rc a > 0 → 機体は Motive +z へ → ROS では -y へ。一貫している。"""
    a = twist_to_tello_rc(0, -0.3, 0, 0)[0]
    assert a > 0
    motion_ros, _ = motive_yup_to_ros((0, 0, +1), IDENTITY)   # Motive +z 方向の移動
    assert motion_ros[1] < 0                                  # = ROS -y（右）= 指令どおり


def test_yaw_helpers():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert abs(yaw_from_quat(*quat_from_yaw(yaw)) - yaw) < 1e-12
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-12
    assert abs(wrap_angle(-0.5 + 4 * math.pi) + 0.5) < 1e-12
