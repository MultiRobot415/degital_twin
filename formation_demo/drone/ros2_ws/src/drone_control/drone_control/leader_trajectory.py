# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""仮想 Leader の状態（位置・速度・進行方向）を作る（ROS 非依存）。

論文 第3章では Leader は楕円軌道を動き、隊形は Leader の進行方向 ṙ_L/|ṙ_L| に合わせて回る（式 3.22）。
ここではその Leader を formation_commander が生成し、/formation/leader（nav_msgs/Odometry）で配信する:
    pose.position         = r_L
    pose.orientation      = 進行方向 ψ（z 軸まわり）
    twist.linear          = ṙ_L [m/s]
    twist.angular.z       = ψ̇ [rad/s]
座標は ROS（x=前, y=左, z=上, world）。
"""

import math
from dataclasses import dataclass
from typing import Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class LeaderState:
    position: Vec3
    velocity: Vec3
    heading: float        # 進行方向 ψ [rad]
    heading_rate: float   # ψ̇ [rad/s]


def heading_from_motion(vx: float, vy: float, ax: float, ay: float,
                        prev_heading: float, threshold: float) -> Tuple[float, float]:
    """速度・加速度から進行方向とその変化率を求める。

    |v| < threshold のときは進行方向を更新しない（直前の値を保持し、変化率 0）。
    論文の式 3.22 は |ṙ| < 0.1 m/s で絶対座標の固定方位 r0 に切り替えるが、切り替えの瞬間に
    隊形が跳ぶのを避けるため、ここでは「直前の進行方向を保持」とした。
    """
    speed2 = vx * vx + vy * vy
    if speed2 < threshold * threshold:
        return prev_heading, 0.0
    return math.atan2(vy, vx), (vx * ay - vy * ax) / speed2   # ψ̇ = (v × a)_z / |v|²


def ellipse_start_point(center: Vec3, heading: float, a: float, b: float) -> Vec3:
    """中心 center の楕円上で、接線（反時計回りの進行方向）が heading になる点。
    Leader をここに置いてから Ellipse(その点, heading, ...) を作ると、楕円の中心が center になる。"""
    phi0 = math.atan2(-math.cos(heading) / a, math.sin(heading) / b)
    return (center[0] + a * math.cos(phi0), center[1] + b * math.sin(phi0), center[2])


class Ellipse:
    """軸が x, y に平行な楕円軌道。開始点と開始時の進行方向が現在の Leader と一致するように置く。

        r(t) = c + (a cos φ(t), b sin φ(t), 0),  φ(t) = φ0 + ω t,  ω = 2π / period（反時計回り）
    """

    def __init__(self, start: Vec3, heading: float, a: float, b: float, period: float):
        self.a, self.b = a, b
        self.omega = 2.0 * math.pi / period
        # 接線 (−a sin φ, b cos φ) が heading の向きになる φ0 を選ぶ
        self.phi0 = math.atan2(-math.cos(heading) / a, math.sin(heading) / b)
        self.center = (start[0] - a * math.cos(self.phi0), start[1] - b * math.sin(self.phi0), start[2])

    def state(self, t: float, prev_heading: float, threshold: float) -> LeaderState:
        phi = self.phi0 + self.omega * t
        c, s, w = math.cos(phi), math.sin(phi), self.omega
        pos = (self.center[0] + self.a * c, self.center[1] + self.b * s, self.center[2])
        vel = (-self.a * w * s, self.b * w * c, 0.0)
        acc = (-self.a * w * w * c, -self.b * w * w * s)
        heading, rate = heading_from_motion(vel[0], vel[1], acc[0], acc[1], prev_heading, threshold)
        return LeaderState(pos, vel, heading, rate)


class GoTo:
    """指定したゴールへ Leader を動かす。
        ① その場でゴールの方向へ向きを変える（回転速度 turn_rate 一定）
        ② ゴールへ直進する（速さ speed 一定）
        ③ final_heading が指定されていれば、その場でその向きへ回る

    隊形は Leader の進行方向に合わせて回るので（式 3.22）、向きを一瞬で変えると隊形が跳び、
    機体の経路が交差しうる（例: 後ろ向きのゴールで左右の機体が入れ替わる）。①③ で向きをゆっくり変えれば
    隊形は形を保ったまま剛体的に回るので、機体同士の経路は交差しない。
    xy の移動が min_xy 未満（高さだけ変える等）のときは ① を省く。
    """

    def __init__(self, start: Vec3, heading: float, goal: Vec3, speed: float, turn_rate: float,
                 final_heading=None, min_xy: float = 0.05, turn: bool = True):
        """turn=False: ①③ を行わず、向きを保ったまま平行移動だけする（隊形も向きを保って平行移動する）。"""
        self.start, self.goal, self.speed, self.turn_rate = start, goal, speed, turn_rate
        dx, dy, dz = (goal[i] - start[i] for i in range(3))
        self.dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.dir = (dx / self.dist, dy / self.dist, dz / self.dist) if self.dist > 0 else (0.0, 0.0, 0.0)
        if not turn:
            final_heading = None
        travel = math.atan2(dy, dx) if turn and math.hypot(dx, dy) >= min_xy else heading
        self.turn1 = (heading, _shortest(travel - heading))                 # (開始角, 回転量)
        h2 = heading + self.turn1[1]
        self.turn2 = (h2, _shortest(final_heading - h2) if final_heading is not None else 0.0)
        self.t1 = abs(self.turn1[1]) / turn_rate
        self.t2 = self.t1 + self.dist / speed
        self.duration = self.t2 + abs(self.turn2[1]) / turn_rate

    def _turn(self, turn, t: float) -> Tuple[float, float]:
        h0, delta = turn
        sign = 1.0 if delta >= 0 else -1.0
        h = h0 + sign * self.turn_rate * t
        return math.atan2(math.sin(h), math.cos(h)), sign * self.turn_rate

    def state(self, t: float, prev_heading: float = 0.0, threshold: float = 0.0) -> LeaderState:
        zero = (0.0, 0.0, 0.0)
        if t < self.t1:                                       # ① 進行方向へ回る
            h, rate = self._turn(self.turn1, t)
            return LeaderState(self.start, zero, h, rate)
        h_travel = math.atan2(math.sin(self.turn2[0]), math.cos(self.turn2[0]))
        if t < self.t2:                                       # ② 直進
            s = (t - self.t1) * self.speed
            pos = tuple(self.start[i] + self.dir[i] * s for i in range(3))
            return LeaderState(pos, tuple(d * self.speed for d in self.dir), h_travel, 0.0)
        if t < self.duration:                                 # ③ 最終の向きへ回る
            h, rate = self._turn(self.turn2, t - self.t2)
            return LeaderState(self.goal, zero, h, rate)
        h_end = self.turn2[0] + self.turn2[1]
        return LeaderState(self.goal, zero, math.atan2(math.sin(h_end), math.cos(h_end)), 0.0)


def _shortest(delta: float) -> float:
    return math.atan2(math.sin(delta), math.cos(delta))
