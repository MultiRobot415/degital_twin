# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""tello_description の URDF を Isaac Sim 用の USD に変換する（1回だけ実行）。

入力 : drone/usd/robots/tello.urdf   （replace.py でテンプレ変数を埋めた版）
出力 : drone/usd/robots/tello.usd    （剛体・質量・衝突つき）

剛体物理を効かせたいので:
    merge_fixed_joints = True   → camera_link を base_link に統合して単一剛体に
    fix_base           = False  → 地面に固定しない（自由に飛ぶ機体）
    import_inertia_tensor=True  → URDF の質量(0.1kg)・慣性をそのまま使う
    （URDF の <collision> 0.18x0.18x0.05 がそのまま当たり判定になる）

実行方法（GUIなしでOK・数分かかる）:
    source /home/utsubo/env_isaaclab/bin/activate
    python ~/research/drone/sim/convert_tello_urdf.py
"""

import argparse

from isaacsim import SimulationApp

# 変換だけなので GUI は不要
parser = argparse.ArgumentParser(description="Convert tello URDF -> USD")
parser.parse_args()
simulation_app = SimulationApp({"headless": True})

import os

import omni.kit.commands

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_IN = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "robots", "tello.urdf"))
USD_OUT = os.path.abspath(os.path.join(THIS_DIR, "..", "usd", "robots", "tello.usd"))


def main():
    assert os.path.exists(URDF_IN), f"URDFが見つからない: {URDF_IN}"
    print(f"[INFO] URDF in : {URDF_IN}")
    print(f"[INFO] USD out: {USD_OUT}")

    # --- インポート設定を作る ---
    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = True      # camera_link を base_link に統合 → 単一剛体
    cfg.fix_base = False               # 自由に飛ぶ機体（地面に固定しない）
    cfg.import_inertia_tensor = True   # URDF の質量・慣性を使う
    cfg.distance_scale = 1.0           # URDF はメートル
    cfg.convex_decomp = False          # 当たり判定は箱なので分解不要

    # --- 変換実行（USD ファイルを書き出す）---
    result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_IN,
        import_config=cfg,
        dest_path=USD_OUT,
    )
    print(f"[INFO] import result: {result}")
    print(f"[DONE] wrote: {USD_OUT}" if os.path.exists(USD_OUT) else "[WARN] USDが出来てないかも")


if __name__ == "__main__":
    main()
    simulation_app.close()
