# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""収集した .npz を可視化：実測値 vs 理論値（公称=単一積分器/速度入力一次系）。

collect_real_rollouts.py が出した x/u/d/rollout_starts/dt を読み、残差 d が何を
意味するかをグラフで確認する。公称モデルは x_{k+1}=x_k+Δt·u（速度がそのまま出る）。
    理論速度 = u            （公称の予測）
    実測速度 = Δx/Δt = u+d  （mocap で測れた実際の速度）
    残差     = d            （その差。CVAE が学習する対象）

出力する図:
  1) 散布図 実測速度 vs 指令 u（軸ごと）。理想線 y=x からの傾き<1 がアンダーシュート。
  2) d のヒストグラム（軸ごと）。バイアス（mean≠0）と広がり（std）を見る。
  3) 時系列（最長ロールアウト）：指令 u（理論）と実測速度を重ねる。差＝d。ラグが見える。
  4) 位置空間（同ロールアウト）：実測位置と一次系予測（指令だけ積分 d=0）を重ねる。
     差＝累積外乱 ΣΔt·d ＝ SHIELD が dt·E[d] で補正している量そのもの。

使い方:
    python plot_rollouts.py --data real_rollouts.npz            # PNG 保存（+画面表示）
    python plot_rollouts.py --data real_rollouts.npz --no-show  # 保存のみ（ヘッドレス）
    ros2 run dist_learn plot_rollouts --data real_rollouts.npz
"""

from __future__ import annotations

import argparse
import os

import numpy as np

# 図のラベルは日本語フォント非依存にするため ASCII で表記
LABELS = ["x (fwd)", "y (left)", "theta (yaw)"]
AX = ["x", "y", "theta"]
UNITS = ["m/s", "m/s", "rad/s"]


def _fit_slope(u_i, d_i):
    """d_i ≈ slope·u_i + b の最小二乗。slope は「指令に比例して失われる割合」。"""
    if np.ptp(u_i) < 1e-9:
        return 0.0, float(np.mean(d_i))
    A = np.vstack([u_i, np.ones_like(u_i)]).T
    slope, b = np.linalg.lstsq(A, d_i, rcond=None)[0]
    return float(slope), float(b)


def make_figures(data, show=True, out_prefix="rollouts"):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = data["x"]
    u = data["u"]
    d = data["d"]
    dt = data["dt"]
    starts = data["rollout_starts"]
    meas_v = u + d  # 実測速度 = Δx/Δt（= u + d）

    saved = []

    # === 図1+2: 散布(実測 vs 指令) と d ヒストグラム（3軸 × 2列）===========
    fig1, axes = plt.subplots(3, 2, figsize=(11, 11))
    for i in range(3):
        slope, b = _fit_slope(u[:, i], d[:, i])

        # --- 左列: measured velocity vs commanded u ---
        ax = axes[i, 0]
        ax.scatter(u[:, i], meas_v[:, i], s=4, alpha=0.25, label="measured")
        lo, hi = u[:, i].min(), u[:, i].max()
        line = np.array([lo, hi])
        ax.plot(line, line, "k--", lw=1.5, label="ideal y=x (nominal 1st-order)")
        # measured fit: meas = (1+slope)*u + b
        ax.plot(line, (1.0 + slope) * line + b, "r-", lw=1.5,
                label=f"fit slope={1.0+slope:.3f}")
        ax.set_xlabel(f"command u_{AX[i]} [{UNITS[i]}]")
        ax.set_ylabel(f"measured d{AX[i]}/dt [{UNITS[i]}]")
        ax.set_title(f"{LABELS[i]}: measured vs nominal")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

        # --- 右列: residual d histogram ---
        ax = axes[i, 1]
        ax.hist(d[:, i], bins=60, alpha=0.8)
        m, s = d[:, i].mean(), d[:, i].std()
        ax.axvline(m, color="r", lw=1.5, label=f"mean={m:+.3f}")
        ax.axvline(0, color="k", lw=1, ls="--")
        ax.set_xlabel(f"residual d_{AX[i]} [{UNITS[i]}]")
        ax.set_ylabel("count")
        ax.set_title(f"{LABELS[i]}: residual dist (std={s:.3f})")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig1.suptitle("measured vs nominal (velocity-input 1st-order) and residual d",
                  fontsize=13)
    fig1.tight_layout(rect=[0, 0, 1, 0.98])
    p1 = f"{out_prefix}_scatter_hist.png"
    fig1.savefig(p1, dpi=120)
    saved.append(p1)

    # === 図3: 最長ロールアウトの時系列（理論 u と実測速度を重ねる）=========
    bounds = list(starts) + [len(x)]
    seg_lens = [bounds[k + 1] - bounds[k] for k in range(len(starts))]
    k = int(np.argmax(seg_lens))
    s0, s1 = bounds[k], bounds[k + 1]
    t = np.cumsum(dt[s0:s1]) - dt[s0]  # ロールアウト内の経過時間

    fig2, axes2 = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i in range(3):
        ax = axes2[i]
        ax.plot(t, u[s0:s1, i], "k--", lw=1.2, label="command u (nominal)")
        ax.plot(t, meas_v[s0:s1, i], "b-", lw=1.0, alpha=0.8, label="measured dx/dt")
        ax.fill_between(t, u[s0:s1, i], meas_v[s0:s1, i], color="r", alpha=0.2,
                        label="residual d")
        ax.set_ylabel(f"{LABELS[i]} [{UNITS[i]}]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes2[-1].set_xlabel("time [s]")
    fig2.suptitle(f"time series (longest rollout #{k}, N={s1-s0}): gap = residual d",
                  fontsize=13)
    fig2.tight_layout(rect=[0, 0, 1, 0.98])
    p2 = f"{out_prefix}_timeseries.png"
    fig2.savefig(p2, dpi=120)
    saved.append(p2)

    # === 図4: 位置空間で「一次系予測 vs 実測」（同じ最長ロールアウト）===========
    # 一次系の公称軌道は指令だけを積分（d=0）: x_nom_{k+1} = x_nom_k + Δt·u_k。
    # 実測位置との差が「累積外乱 Σ Δt·d」＝ SHIELD が dt·E[d] で補正している量そのもの。
    x_meas = x[s0:s1]
    x_nom = np.empty_like(x_meas)
    x_nom[0] = x_meas[0]  # ロールアウト開始点で実測と一致させる
    for j in range(1, len(x_meas)):
        x_nom[j] = x_nom[j - 1] + dt[s0 + j - 1] * u[s0 + j - 1]

    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i in range(3):
        ax = axes3[i]
        ax.plot(t, x_meas[:, i], "b-", lw=1.2, label="measured x (mocap)")
        ax.plot(t, x_nom[:, i], "k--", lw=1.2,
                label="nominal 1st-order (integrate u, d=0)")
        ax.fill_between(t, x_nom[:, i], x_meas[:, i], color="r", alpha=0.2,
                        label="accumulated drift  Σ Δt·d")
        pos_unit = "rad" if i == 2 else "m"
        ax.set_ylabel(f"{LABELS[i]} [{pos_unit}]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes3[-1].set_xlabel("time [s]")
    fig3.suptitle(
        f"position space (longest rollout #{k}, N={s1-s0}): "
        f"measured vs 1st-order prediction (gap = what SHIELD corrects)",
        fontsize=13)
    fig3.tight_layout(rect=[0, 0, 1, 0.98])
    p3 = f"{out_prefix}_position.png"
    fig3.savefig(p3, dpi=120)
    saved.append(p3)

    print("[plot] 保存:", ", ".join(saved))
    if show:
        plt.show()
    return saved


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="収集 .npz の可視化（実測 vs 理論一次系）")
    p.add_argument("--data", required=True, help="collect が出した .npz")
    p.add_argument("--no-show", action="store_true", help="画面表示せず PNG 保存のみ")
    args = p.parse_args(argv)

    data = np.load(args.data)
    prefix = os.path.splitext(args.data)[0]
    make_figures(data, show=not args.no_show, out_prefix=prefix)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
