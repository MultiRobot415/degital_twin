# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フェーズ2 / ステップ1：アリーナを読み込んで Tello役の箱を1機ホバリングさせる。

このスクリプトがやること（それだけ）:
  1. Isaac Sim を起動する
  2. 自作した drone_arena.usd（3x4m フィールド + 床 + 柱 + ライト）を開く
  3. Tello に見立てた箱を1個、中央の高さ1mに「浮かせて」置く
  4. ただ静止させたまま、シミュレーションを回し続ける

物理で落下しないように、箱は VisualCuboid（物理なし）で作る。
＝「キネマティックに位置を指定する」方式。次のステップでこの位置を
速度で毎フレーム更新すると、そのまま①速度モデルになる。

実行方法（Isaac Lab の venv の python で直接！colcon ではない）:

    source /home/utsubo/env_isaaclab/bin/activate
    python ~/research/drone/sim/step1_hover.py

オプション:
    --headless    GUI を出さずに動かす（動作確認だけしたいとき）
"""

# =============================================================================
# 1) 何よりも先に Isaac Sim アプリを起動する。
#    これより前に isaacsim.core 等を import してはいけない（まだ存在しないため）。
# =============================================================================
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Step1: load arena and hover a drone")
parser.add_argument("--headless", action="store_true", help="GUIを出さない")
args_cli = parser.parse_args()

# SimulationApp を作った瞬間に Isaac Sim が起動し、core モジュールが import 可能になる
simulation_app = SimulationApp({"headless": args_cli.headless})

# =============================================================================
# 2) ここから下でだけ isaacsim.core を import できる
# =============================================================================
import os

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.stage import open_stage

# --- 自作アリーナ USD の場所（このスクリプトからの相対で解決）---------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARENA_USD = os.path.join(THIS_DIR, "..", "usd", "env", "drone_arena.usd")
ARENA_USD = os.path.abspath(ARENA_USD)

# --- ホバリングさせる位置（フィールド中央・高さ1m）-------------------------
#   Isaac Sim の API は numpy 配列を期待するので np.array で渡す
HOVER_POS = np.array([0.0, 0.0, 1.0])  # x, y, z [m]


def main():
    # --- アリーナを「現在のステージ」として開く ---------------------------
    # これで床・柱・ライト・PhysicsScene がそのまま読み込まれる
    print(f"[INFO] opening arena: {ARENA_USD}")
    assert os.path.exists(ARENA_USD), f"USDが見つからない: {ARENA_USD}"
    open_stage(ARENA_USD)

    # --- World で今のステージを包む（物理・描画ループの管理役）-----------
    world = World()

    # --- Tello役の箱を1個スポーン -----------------------------------------
    #   VisualCuboid = 見た目だけの箱（重力で落ちない）。
    #   サイズは Tello っぽく 18cm x 18cm x 4cm の平たい箱に。
    drone = VisualCuboid(
        prim_path="/World/Tello",
        name="tello",
        position=HOVER_POS,
        scale=np.array([0.18, 0.18, 0.04]),
        color=np.array([0.1, 0.6, 1.0]),  # 水色っぽく（見つけやすいよう）
    )

    # --- 初期化（ここでシーンが「再生」できる状態になる）------------------
    world.reset()
    print("[INFO] setup complete. drone is hovering at", HOVER_POS)

    # --- メインループ：ただ静止させたまま回し続ける ----------------------
    #   いまは何もしない＝速度ゼロ＝その場でホバリング。
    #   次のステップで、ここに「位置を速度で更新する」処理を足す。
    while simulation_app.is_running():
        # 念のため毎フレーム、ホバリング位置に置き直す（位置が動かないことを保証）
        drone.set_world_pose(position=HOVER_POS)
        world.step(render=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
