# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フェーズ2 / ステップ2：キーボードの速度指令で Tello役の箱を飛ばす（①+傾き演出）。

このスクリプトがやること:
  1. アリーナを読み込み、Tello役の箱を1機ホバリングさせる（step1 と同じ土台）
  2. キーボード入力を「機体座標の速度指令(vx, vy, vz, ヨー角速度)」として受け取る
  3. その速度で毎フレーム位置を積分して動かす（=①速度モデル）
  4. 速度に応じて機体を見た目だけ傾ける（=傾き演出）

速度モデルの中身（ここが本体・入力源はあとでROS2に差し替え可能）:
    yaw      += wz * dt
    world_v   = yawで回した [vx, vy, vz]
    position += world_v * dt
    pitch = -k * vx   (前進で前傾) / roll = k * vy (横移動でバンク)

キー割り当て:
    W / S : 前進 / 後退        A / D : 左 / 右
    R / F : 上昇 / 下降        Q / E : 左旋回 / 右旋回
    （キーを離すとその軸は0へ＝停止）
    ※ Isaac Sim のウィンドウにフォーカスを当てた状態で操作すること

実行方法:
    source /home/utsubo/env_isaaclab/bin/activate
    python ~/research/drone/sim/step2_keyboard_velocity.py
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim を起動
# =============================================================================
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Step2: keyboard velocity teleop")
parser.add_argument("--headless", action="store_true", help="GUIを出さない")
args_cli = parser.parse_args()

simulation_app = SimulationApp({"headless": args_cli.headless})

# =============================================================================
# 2) ここから core / carb を import できる
# =============================================================================
import os

import numpy as np

import carb
import omni.appwindow
from carb.input import KeyboardEventType, KeyboardInput

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import open_stage

# --- アリーナ USD の場所 ----------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARENA_USD = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "env", "drone_arena.usd"))

# --- 動きのパラメータ（いじって感触を調整する場所）-------------------------
START_POS = np.array([0.0, 0.0, 1.0])  # 初期ホバリング位置 [m]

MAX_VXY = 1.0     # 前後・左右の最大速度 [m/s]
MAX_VZ = 0.6      # 上下の最大速度 [m/s]
MAX_WZ = 1.2      # ヨー最大角速度 [rad/s]
SMOOTH = 0.1      # 指令への追従の鈍さ（0=即時, 小さいほどヌルッと。実機の慣性っぽさ）

TILT_GAIN = 0.30  # 速度→傾き角[rad]のゲイン（大きいほど大きく傾く演出）

# --- アリーナ内に収めるための簡易クランプ範囲 -------------------------------
X_LIM, Y_LIM = 1.4, 1.9
Z_MIN, Z_MAX = 0.1, 2.0


class KeyboardVelocity:
    """押されているキーから、機体座標の速度指令(vx, vy, vz, wz)を作る。"""

    def __init__(self):
        self.pressed = set()
        # Isaac Sim のウィンドウのキーボードイベントを購読する
        appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def _on_keyboard_event(self, event, *args):
        # キーが押された/離されたら、押下中セットを更新するだけ
        if event.type == KeyboardEventType.KEY_PRESS:
            self.pressed.add(event.input)
        elif event.type == KeyboardEventType.KEY_RELEASE:
            self.pressed.discard(event.input)
        return True  # イベントを消費

    def command(self):
        """現在の押下状態 → 速度指令 [vx, vy, vz, wz] を返す。"""
        K = KeyboardInput
        p = self.pressed

        vx = (K.W in p) * MAX_VXY - (K.S in p) * MAX_VXY  # 前後
        vy = (K.A in p) * MAX_VXY - (K.D in p) * MAX_VXY  # 左右
        vz = (K.R in p) * MAX_VZ - (K.F in p) * MAX_VZ    # 上下
        wz = (K.Q in p) * MAX_WZ - (K.E in p) * MAX_WZ    # 旋回
        return np.array([vx, vy, vz, wz])


def main():
    # --- アリーナ読み込み + World ----------------------------------------
    assert os.path.exists(ARENA_USD), f"USDが見つからない: {ARENA_USD}"
    print(f"[INFO] opening arena: {ARENA_USD}")
    open_stage(ARENA_USD)
    world = World()

    # --- Tello役の箱（step1 と同じ）--------------------------------------
    drone = VisualCuboid(
        prim_path="/World/Tello",
        name="tello",
        position=START_POS,
        scale=np.array([0.18, 0.18, 0.04]),
        color=np.array([0.1, 0.6, 1.0]),
    )

    world.reset()
    keys = KeyboardVelocity()
    print("[INFO] ready. WSAD=平面移動 / RF=上下 / QE=旋回（ウィンドウにフォーカス）")

    # --- 機体の状態（位置と向きを自前で積分する）-------------------------
    pos = START_POS.copy().astype(float)
    yaw = 0.0
    vel = np.zeros(3)  # なめらかに追従させるための「現在速度」
    dt = world.get_physics_dt()

    while simulation_app.is_running():
        # 1) 入力 → 速度指令（機体座標）
        cmd = keys.command()
        cmd_v_body = cmd[:3]   # [vx, vy, vz]
        wz = cmd[3]            # ヨー角速度

        # 2) 指令速度へなめらかに追従（実機の慣性っぽさ）
        vel += (cmd_v_body - vel) * SMOOTH

        # 3) ヨーを更新
        yaw += wz * dt

        # 4) 機体座標の速度を世界座標へ回す（ヨーだけ考慮）
        c, s = np.cos(yaw), np.sin(yaw)
        world_v = np.array([
            c * vel[0] - s * vel[1],
            s * vel[0] + c * vel[1],
            vel[2],
        ])

        # 5) 位置を積分し、アリーナ内に収める
        pos += world_v * dt
        pos[0] = np.clip(pos[0], -X_LIM, X_LIM)
        pos[1] = np.clip(pos[1], -Y_LIM, Y_LIM)
        pos[2] = np.clip(pos[2], Z_MIN, Z_MAX)

        # 6) 傾き演出：速度から roll / pitch を逆算（物理ではなく見た目）
        pitch = -TILT_GAIN * vel[0]   # 前進で前傾
        roll = TILT_GAIN * vel[1]     # 横移動でバンク
        quat = euler_angles_to_quat(np.array([roll, pitch, yaw]))  # (w,x,y,z)

        # 7) 反映して1ステップ進める
        drone.set_world_pose(position=pos, orientation=quat)
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
