# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""SHIELD フィルタ実行ログ（軌跡・h・クリアランス）を可視化する。

shield_filter_node が出した log/traj/<日付>/<時刻>_<mode>.npz を読み、
sim の run_shield.py と同じ観点でプロットする:

  1) xy 軌跡 + 障害物（本体円）+ 安全境界 R_total。sdf<0（侵入）点は赤で強調。
  2) h(t)：確率的バリア値。h=0 が安全境界（h>0 で安全側）。
  3) クリアランス sdf(t) = 障害物表面までの符号付き距離。0 を割ると衝突。
  4) フィルタ介入：|u_nom| と |u_safe| の時系列（フィルタがどれだけ指令を曲げたか）。

使い方:
    python plot_shield_run.py --data log/traj/2026-06-18/153012_shield.npz
    python plot_shield_run.py --data <npz> --no-show       # 保存のみ
    ros2 run dist_learn plot_shield_run --data <npz>
    # 省略時は log/traj/ 配下の最新を自動で拾う
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np


def _latest() -> str | None:
    cands = sorted(glob.glob(os.path.expanduser(
        "~/research/drone/ros2_ws/log/traj/*/*.npz")))
    return cands[-1] if cands else None


def make_figure(data, show=True, out_prefix="shield_run"):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = data["t"]
    x = data["x"]
    u_nom = data["u_nom"]
    u_safe = data["u_safe"]
    h = data["h"]
    sdf = data["sdf"]
    obstacles = data["obstacles"]            # (M,3) = [ox,oy,r]
    robot_radius = float(data["robot_radius"])
    mode = str(data["mode"]) if "mode" in data.files else "?"

    unsafe = sdf < 0.0
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # 1) xy trajectory + obstacles
    ax = axes[0, 0]
    th = np.linspace(0, 2 * np.pi, 100)
    for ox, oy, r in obstacles:
        ax.fill(ox + (r + robot_radius) * np.cos(th),
                oy + (r + robot_radius) * np.sin(th),
                color="hotpink", alpha=0.20, zorder=0)        # safety margin R_total
        ax.fill(ox + r * np.cos(th), oy + r * np.sin(th),
                color="crimson", alpha=0.75, zorder=1)         # obstacle body
    ax.plot(x[:, 0], x[:, 1], "-", color="royalblue", lw=1.5, label="trajectory")
    ax.plot(x[0, 0], x[0, 1], "go", ms=8, label="start")
    ax.plot(x[-1, 0], x[-1, 1], "ks", ms=7, label="end")
    if unsafe.any():
        ax.plot(x[unsafe, 0], x[unsafe, 1], "rx", ms=6, label="sdf<0 (collision)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_xlabel("px [m]"); ax.set_ylabel("py [m]")
    ax.set_title(f"trajectory ({mode})")

    # 2) h(t)
    ax = axes[0, 1]
    ax.plot(t, h, "-", color="forestgreen", lw=1.4)
    ax.axhline(0, color="k", ls="--", lw=1, label="h=0 (safety boundary)")
    ax.set_xlabel("time [s]"); ax.set_ylabel("h (barrier)")
    ax.set_title("barrier value h(t)  — h>0 = safe"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # 3) clearance sdf(t)
    ax = axes[1, 0]
    ax.plot(t, sdf, "-", color="darkorange", lw=1.4)
    ax.axhline(0, color="r", ls="--", lw=1, label="sdf=0 (contact)")
    ax.fill_between(t, sdf, 0, where=(sdf < 0), color="red", alpha=0.3)
    ax.set_xlabel("time [s]"); ax.set_ylabel("sdf [m]")
    ax.set_title("clearance to obstacle surface"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # 4) filter intervention |u_nom| vs |u_safe|
    ax = axes[1, 1]
    ax.plot(t, np.linalg.norm(u_nom, axis=1), "-", color="gray", lw=1.2, label="|u_nom|")
    ax.plot(t, np.linalg.norm(u_safe, axis=1), "-", color="purple", lw=1.4, label="|u_safe|")
    ax.set_xlabel("time [s]"); ax.set_ylabel("speed [m/s]")
    ax.set_title("filter intervention (nominal vs safe)"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.suptitle(f"SHIELD run: {mode}  (N={len(t)} steps, "
                 f"min sdf={sdf.min():.3f} m, unsafe={int(unsafe.sum())} steps)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = f"{out_prefix}.png"
    fig.savefig(out, dpi=120)
    print(f"[plot] 保存: {out}")
    if show:
        plt.show()
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="SHIELD 実行ログの可視化（軌跡・h・sdf）")
    p.add_argument("--data", default=None, help="npz（省略時 log/traj/ の最新）")
    p.add_argument("--no-show", action="store_true", help="画面表示せず PNG 保存のみ")
    args = p.parse_args(argv)

    data_path = args.data or _latest()
    if data_path is None:
        print("[plot] log/traj/ に npz が見つからない。--data で指定してください。")
        return 1
    data = np.load(data_path, allow_pickle=False)
    make_figure(data, show=not args.no_show, out_prefix=os.path.splitext(data_path)[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
