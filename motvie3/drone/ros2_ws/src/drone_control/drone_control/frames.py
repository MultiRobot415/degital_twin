# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""座標系・符号の変換を「ここだけ」に集めたモジュール（最重要）。

このプロジェクトには 3 つの座標規約が混在する。変換はシステムの境界
（入口 = pose_bridge / swarm_sim の mocap 模擬、出口 = tello_swarm_driver）でだけ行い、
制御ノード（formation_agent / position_controller）は ROS 規約しか見ない。

  ① Motive（OptiTrack, Y-up 配信・右手系）  … ref/mocap_stack の座標もこれ
        x = 前, y = 上, z = 右
        （ref の実機ログで「rc_lr > 0（右）で mocap_z が増える」ことを確認済み）
  ② ROS（REP-103, Z-up・右手系）            … /telloN/pose, formation の offset はすべてこれ
        x = 前, y = 左, z = 上,  yaw は +z まわり反時計回りが正
  ③ Tello SDK の rc コマンド  "rc a b c d"   … 各 -100〜100
        a = 左右（右が正）, b = 前後（前が正）, c = 上下（上が正）, d = ヨー（時計回りが正）

変換:
  ①→② 位置   (x, y, z)_M        → (x, -z, y)_ROS
       姿勢   (qx, qy, qz, qw)_M → (qx, -qz, qy, qw)_ROS
       = X 軸まわり +90° の回転 C による共役 R_ROS = C R_M C^T。
       剛体を「機首 +X・Motive の世界軸に揃えた状態」で作成していれば、
       機体座標も同じ規約で読み替わり、ROS 側で yaw が正しく取り出せる。
  ②→③ rc = (-100*linear.y, 100*linear.x, 100*linear.z, -100*angular.z)
       （tello_ros/tello_driver と同じ式。左右とヨーは符号が反転する）

ref のコードの値（目標位置・隊形オフセット）を流用するときは、必ず ①→② を通すこと。
例: ref の 2号機オフセット (-0.4, 0, +0.4)_M → (-0.4, -0.4, 0)_ROS（リーダーの後ろ・右）。
"""

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w)

RC_LIMIT = 100


# -----------------------------------------------------------------------------
# ① Motive (Y-up)  <->  ② ROS (Z-up)
# -----------------------------------------------------------------------------
def motive_yup_to_ros(p: Vec3, q: Quat) -> Tuple[Vec3, Quat]:
    """Motive(Y-up) の位置・姿勢を ROS(Z-up) に変換する。"""
    return (p[0], -p[2], p[1]), (q[0], -q[2], q[1], q[3])


def ros_to_motive_yup(p: Vec3, q: Quat) -> Tuple[Vec3, Quat]:
    """motive_yup_to_ros の逆変換（シミュレータで mocap を模擬するときに使う）。"""
    return (p[0], p[2], -p[1]), (q[0], q[2], -q[1], q[3])


# -----------------------------------------------------------------------------
# ② ROS の速度指令  ->  ③ Tello rc
# -----------------------------------------------------------------------------
def _round_half_away(v: float) -> int:
    """C/C++ の round と同じ「0 から遠い側へ丸める」。Python の round は偶数丸めなので使わない。"""
    return int(math.copysign(math.floor(abs(v) + 0.5), v))


def twist_to_tello_rc(linear_x: float, linear_y: float, linear_z: float,
                      angular_z: float) -> Tuple[int, int, int, int]:
    """正規化 cmd_vel（機体座標・ROS 規約, 各 -1〜1）を Tello の rc (a, b, c, d) に変換する。

    入力は [-1, 1] にクランプしてから 100 倍する（Tello の受け付け範囲 -100〜100）。
    """
    def scale(v: float) -> int:
        v = max(-1.0, min(1.0, float(v)))
        return max(-RC_LIMIT, min(RC_LIMIT, _round_half_away(v * RC_LIMIT)))

    return (scale(-linear_y),   # a: 右が正 ← ROS は左が正
            scale(linear_x),    # b: 前が正
            scale(linear_z),    # c: 上が正
            scale(-angular_z))  # d: 時計回りが正 ← ROS は反時計回りが正


def format_rc(rc: Tuple[int, int, int, int]) -> str:
    return f"rc {rc[0]} {rc[1]} {rc[2]} {rc[3]}"


# -----------------------------------------------------------------------------
# 角度ユーティリティ（ROS 規約）
# -----------------------------------------------------------------------------
def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """クォータニオン → ヨー角[rad]（+z まわり、反時計回り正）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Quat:
    """ヨー角[rad] → クォータニオン (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle(a: float) -> float:
    """角度を (-pi, pi] に折り返す。"""
    return math.atan2(math.sin(a), math.cos(a))
