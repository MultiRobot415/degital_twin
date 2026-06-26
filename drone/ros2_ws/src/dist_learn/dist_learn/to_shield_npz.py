# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""実機収集 npz(3D) → SHIELD 再現リポジトリの 2D 形式へ変換する。

collect_real_rollouts.py の出力は 3D（x=[px,py,θ], u=[vx,vy,ω], d=[dx,dy,dθ]）。
一方 SHIELD 再現リポジトリ(202605_SHIELD)の CVAE 学習は完全に 2D
（d_dim=state_dim=action_dim=2）。yaw を今は切り捨て、xy だけにスライスして
学習側が読める `data/<name>.npz` を作る。

学習側は data["x"], data["u"], data["d"] を読むだけなので、キー名はそのまま。
**rollout_starts は残す**：境界を跨がない窓化（次のパッチ）で使うため
（残しても train_cvae.py は無視するので無害）。

使い方:
    # 最新ログを自動で拾って SHIELD の data/ に real_xy.npz を作る
    python to_shield_npz.py
    # 明示指定
    python to_shield_npz.py --in log/residual/2026-06-15/110424_rollouts.npz \\
        --out ~/research/202605_SHIELD/shield_reproduction/data/real_xy.npz
    ros2 run dist_learn to_shield_npz --in <npz> --out <npz>

そのあと（リポジトリ側で）:
    python scripts/cvae/train_cvae.py --data real_xy.npz --out cvae_real.pth
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np

# SHIELD リポジトリの data ディレクトリ（存在すれば既定の出力先に使う）。
# 注意: SHIELD は別アカウント(taisei-uuu)の独立リポジトリなので、degital_twin と
# git 管理が混ざらないよう research の **外**（~/202605_SHIELD）に置いている。
DEFAULT_SHIELD_DATA = os.path.expanduser(
    "~/202605_SHIELD/shield_reproduction/data"
)


def _latest_log() -> str | None:
    """log/residual/<日付>/<時刻>_rollouts.npz のうち最新を返す。"""
    cands = sorted(glob.glob("log/residual/*/*_rollouts.npz"))
    return cands[-1] if cands else None


def convert(in_path: str, out_path: str) -> None:
    src = np.load(in_path)
    x, u, d = src["x"], src["u"], src["d"]
    if x.shape[1] < 2:
        raise ValueError(f"入力 x の次元が 2 未満: {x.shape}")

    # xy だけにスライス（θ/ω/dθ は捨てる）
    x2, u2, d2 = x[:, :2], u[:, :2], d[:, :2]

    out = {"x": x2, "u": u2, "d": d2}
    # 境界情報は残す（次の境界安全な窓化で使う。学習側は無視するので無害）
    if "rollout_starts" in src.files:
        out["rollout_starts"] = src["rollout_starts"]

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    np.savez(out_path, **out)

    np.set_printoptions(precision=4, suppress=True)
    print(f"[convert] {in_path}  (3D, N={len(x)})")
    print(f"       -> {os.path.abspath(out_path)}  (2D, N={len(x2)})")
    print(f"   x2 shape={x2.shape}  u2 shape={u2.shape}  d2 shape={d2.shape}")
    print(f"   d(xy) mean={d2.mean(0)}  std={d2.std(0)}")
    if "rollout_starts" in out:
        rs = out["rollout_starts"]
        print(f"   rollout_starts: {len(rs)} 本（保持）= {rs.tolist()}")
    else:
        print("   rollout_starts: 入力に無し（窓は境界を考慮せず滑る点に注意）")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="実機 3D npz → SHIELD 2D npz 変換")
    p.add_argument("--in", dest="in_path", default=None,
                   help="入力（未指定なら log/residual/ の最新 _rollouts.npz）")
    p.add_argument("--out", dest="out_path", default=None,
                   help=f"出力（未指定なら {DEFAULT_SHIELD_DATA}/real_xy.npz）")
    p.add_argument("--name", default="real_xy.npz",
                   help="既定出力先でのファイル名（既定 real_xy.npz）")
    args = p.parse_args(argv)

    in_path = args.in_path or _latest_log()
    if in_path is None:
        print("[convert] 入力が見つからない（log/residual/ に *_rollouts.npz が無い）。"
              "--in で明示してください。")
        return 1

    out_path = args.out_path or os.path.join(DEFAULT_SHIELD_DATA, args.name)
    convert(in_path, out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
