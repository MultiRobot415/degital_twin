# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""3機クロス実機ログの可視化：軌跡オーバーレイ + 全機の h(t) を2枚横並びで出す。

log/traj/<日付>/<時刻>_telloN_shield_filter_<mode>.npz を3機ぶん読み、
同じフォルダに <時刻>_<mode>.png を書く（過去の作図と同じ置き場所・命名）。

作図スタイル（ユーザー定番）:
    フォントは Times New Roman（Linux は Liberation Serif で代替）。
    静止障害物は本体 crimson alpha0.75 + 安全マージン hotpink alpha0.20。
    start=白丸黒縁 / end=黒四角。タイトル・grid・カラーバー・会場枠は描かない。
    系列色は検証済みリファレンスパレットの categorical slot 1-3（青/オレンジ/アクア）。
    slot3(aqua) は明色面でコントラストが 3:1 未満なので relief ルールに従い
    全系列に直接ラベルを付ける（凡例だけに頼らない）。

使い方:
    ~/env_isaaclab/bin/python -m dist_learn.plot_cross3 --run 141221
    ros2 run dist_learn plot_cross3 --run 143101 --day 2026-08-31
"""
import argparse
import glob
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

INK, INK2 = "#0b0b0b", "#52514e"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]      # categorical slot 1,2,3
OBS_BODY, OBS_MARGIN = "crimson", "hotpink"
DEFAULT_ROOT = os.path.expanduser("~/research/drone/ros2_ws/log/traj")

# 軌跡パネルの直接ラベルを置くアンカー（軌跡が空いている場所を通る点に寄せる）と
# そこからのテキストオフセット。会場や S/G を変えたらここを調整する。
# 場内に収まった回で共通の framing にするための最低レンジ [x_min,x_max,y_min,y_max]。
BASE_BOX = (-0.95, 0.95, -0.95, 0.95)

# 論文に貼ると縮小されるので既定を大きめ・太めにしてある。--fontscale で全体倍率、
# --linescale で線と マーカーの倍率を変えられる。
FS = dict(label=21, tick=16, legend=16, dlabel=17, note=14)
LW = dict(series=2.6, halo=5.4, zero=1.4, spine=1.4, start=11.0, end=10.0)


def _scale(fontscale=1.0, linescale=1.0):
    for k in FS:
        FS[k] = FS[k] * fontscale
    for k in LW:
        LW[k] = LW[k] * linescale
ANCHOR = {1: (0.72, 0.25), 2: (-0.42, 0.05), 3: (0.15, -0.86)}
OFFSET = {1: (0.07, 0.05), 2: (-0.34, 0.04), 3: (-0.12, -0.15)}


def _style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Liberation Serif", "Nimbus Roman",
                       "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "axes.edgecolor": INK, "axes.labelcolor": INK, "text.color": INK,
        "figure.facecolor": "white", "axes.facecolor": "white",
    })


def load(run, day, root):
    d0 = os.path.join(root, day)
    runs, mode = {}, None
    for i in (1, 2, 3):
        fs = glob.glob(os.path.join(d0, f"{run}_tello{i}_*.npz"))
        if not fs:
            raise FileNotFoundError(f"{d0}/{run}_tello{i}_*.npz が無い")
        d = np.load(sorted(fs)[0], allow_pickle=True)
        runs[i] = dict(t=d["t"], x=d["x"], h=d["h"], sdf=d["sdf"],
                       obs=d["obstacles"], rr=float(d["robot_radius"]),
                       slack=d["slack"])
        mode = str(d["mode"])
    return runs, mode, d0


def plot(runs, mode, out):
    _style()
    # obstacle_bridge の並びは [相手機..., 静止障害物...]。相手機は動くので末尾2件
    # （会場に置いた ID4/ID5）だけを静止障害物として描く。
    statics = runs[1]["obs"][-2:]
    robot_r = runs[1]["rr"]

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.8, 6.0))

    # ---------- 左：軌跡 ----------
    for cx, cy, r in statics:
        axL.add_patch(Circle((cx, cy), r + robot_r, facecolor=OBS_MARGIN,
                             alpha=0.20, edgecolor="none", zorder=1))
        axL.add_patch(Circle((cx, cy), r, facecolor=OBS_BODY, alpha=0.75,
                             edgecolor="none", zorder=2))
    any_bad = False
    for i in (1, 2, 3):
        x, sdf = runs[i]["x"], runs[i]["sdf"]
        axL.plot(x[:, 0], x[:, 1], color=SERIES[i - 1], lw=LW["series"], zorder=3,
                 solid_capstyle="round")
        # sdf<0（＝必要クリアランスを割った）区間を黒のハローで強調する。
        # 色だけに頼らないため凡例にも項目を出す（status は色単独で示さない規則）。
        bad = np.asarray(sdf) < 0
        if bad.any():
            any_bad = True
            idx = np.where(bad)[0]
            for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                if len(seg) < 2:
                    continue
                axL.plot(x[seg, 0], x[seg, 1], color=INK, lw=LW["halo"], zorder=3.5,
                         solid_capstyle="round")
                axL.plot(x[seg, 0], x[seg, 1], color=SERIES[i - 1],
                         lw=LW["series"] * 1.15, zorder=3.6, solid_capstyle="round")
    for i in (1, 2, 3):
        x = runs[i]["x"]
        axL.plot(x[0, 0], x[0, 1], "o", ms=LW["start"], mfc="white", mec=INK,
                 mew=1.8, zorder=5)
        axL.plot(x[-1, 0], x[-1, 1], "s", ms=LW["end"], mfc=INK, mec=INK, zorder=5)
    _dlabels = []                           # レンジ確定後に描く（クランプのため）
    for i in (1, 2, 3):
        x = runs[i]["x"]
        k = int(np.argmin(np.linalg.norm(x - np.array(ANCHOR[i]), axis=1)))
        dx, dy = OFFSET[i]
        _dlabels.append((i, (x[k, 0], x[k, 1]), (x[k, 0] + dx, x[k, 1] + dy)))

    axL.set_xlabel(r"$x$ [m]", fontsize=FS["label"], fontweight="bold")
    axL.set_ylabel(r"$y$ [m]", fontsize=FS["label"], fontweight="bold")
    axL.set_aspect("equal", adjustable="box")
    # レンジはデータ（軌跡＋障害物のマージン円）を全部入れる正方形。固定値だと
    # 場外へ逃げた回で軌跡が切れて事故が見えなくなるため。
    # 標準枠 BASE_BOX は必ず含める。こうすると場内で収まった回は毎回同じ framing に
    # なり複数回の図を並べて比較できる。場外へ逃げた回だけレンジが広がる。
    px = np.concatenate([runs[i]["x"][:, 0] for i in (1, 2, 3)]
                        + [statics[:, 0] - statics[:, 2] - robot_r,
                           statics[:, 0] + statics[:, 2] + robot_r,
                           np.array(BASE_BOX[:2])])
    py = np.concatenate([runs[i]["x"][:, 1] for i in (1, 2, 3)]
                        + [statics[:, 1] - statics[:, 2] - robot_r,
                           statics[:, 1] + statics[:, 2] + robot_r,
                           np.array(BASE_BOX[2:])])
    cx, cy = (px.min() + px.max()) / 2, (py.min() + py.max()) / 2
    half = max(px.max() - px.min(), py.max() - py.min()) / 2 + 0.18
    axL.set_xlim(cx - half, cx + half)
    axL.set_ylim(cy - half, cy + half)
    tk = lambda lo, hi: np.arange(np.ceil(lo / 0.5) * 0.5, hi + 1e-9, 0.5)
    axL.set_xticks(tk(cx - half, cx + half))
    axL.set_yticks(tk(cy - half, cy + half))
    # 直接ラベル（テキストは黒＝text token）。軸の外へ出ないようクランプする。
    x0, x1 = axL.get_xlim()
    y0, y1 = axL.get_ylim()
    for i, anchor, txt in _dlabels:
        tx = min(max(txt[0], x0 + 0.03 * (x1 - x0)), x1 - 0.30 * (x1 - x0))
        ty = min(max(txt[1], y0 + 0.04 * (y1 - y0)), y1 - 0.04 * (y1 - y0))
        axL.annotate(f"Agent {i}", xy=anchor, xytext=(tx, ty), color=INK,
                     fontsize=FS["dlabel"], fontweight="bold", zorder=6)

    axL.tick_params(colors=INK, labelsize=FS["tick"], width=LW["spine"], length=6)
    for lb in axL.get_xticklabels() + axL.get_yticklabels():
        lb.set_fontweight("bold")
    for s in ("top", "right"):
        axL.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        axL.spines[s].set_linewidth(LW["spine"])
    # 左の凡例はマーカーの意味だけ。軸の外に置いて軌跡と重ねない。
    pr = [plt.Line2D([], [], marker="o", ls="none", ms=LW["start"], mfc=OBS_BODY,
                     mec="none", alpha=0.75),
          plt.Line2D([], [], marker="o", ls="none", ms=LW["start"], mfc="white",
                     mec=INK, mew=1.8),
          plt.Line2D([], [], marker="s", ls="none", ms=LW["end"], mfc=INK, mec=INK)]
    # 黒四角は「軌跡の終端」。ゴールに届かなかった回もあるので Goal とは書かない。
    lb = ["Static obstacle", "Start", "End"]
    if any_bad:
        pr.append(plt.Line2D([], [], color=INK, lw=LW["halo"]))
        lb.append(r"$h<0$ (clearance violated)")
    axL.legend(pr, lb, loc="upper center", bbox_to_anchor=(0.5, -0.16),
               ncol=2 if len(lb) > 3 else len(lb), frameon=False,
               prop={"weight": "bold", "size": FS["legend"]},
               labelcolor=INK, handletextpad=0.6, columnspacing=1.6,
               borderaxespad=0.0)

    # ---------- 右：h(t) ----------
    for i in (1, 2, 3):
        t, h, sdf = runs[i]["t"], runs[i]["h"], runs[i]["sdf"]
        axR.plot(t, h, color=SERIES[i - 1], lw=LW["series"], solid_capstyle="round",
                 zorder=3, label=f"Agent {i}")
        bad = np.asarray(sdf) < 0
        if bad.any():
            idx = np.where(bad)[0]
            for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                if len(seg) < 2:
                    continue
                axR.plot(t[seg], h[seg], color=INK, lw=LW["halo"], zorder=3.5,
                         solid_capstyle="round")
                axR.plot(t[seg], h[seg], color=SERIES[i - 1],
                         lw=LW["series"] * 1.15, zorder=3.6, solid_capstyle="round")
    axR.axhline(0.0, color=INK2, lw=LW["zero"], zorder=2)
    tmax = max(runs[i]["t"][-1] for i in (1, 2, 3))
    hmax = max(np.nanmax(runs[i]["h"]) for i in (1, 2, 3))
    hmin = min(np.nanmin(runs[i]["h"]) for i in (1, 2, 3))
    lo, hi = min(hmin, 0.0) - 0.03 * (hmax - min(hmin, 0.0)), hmax * 1.10
    # 注記は線と重なりうるので白の下地を敷いて読めるようにする（枠線は付けない）
    axR.annotate(r"safety boundary  $h=0$", xy=(tmax * 0.02, 0.0),
                 xytext=(tmax * 0.02, 0.02 * (hi - lo)), color=INK2,
                 fontsize=FS["note"], fontweight="bold", zorder=7,
                 bbox=dict(facecolor="white", edgecolor="none", pad=1.5, alpha=0.85))
    # 終端ラベル：h の終値が近いと重なるので、必要な分だけ縦にずらして引き出し線を引く。
    # 隙間は文字高さから見積もる（軸の高さ ≈ figure高さ*0.72*72pt）。
    gap = 1.45 * FS["dlabel"] / (6.0 * 0.72 * 72.0) * (hi - lo)
    ends = sorted(((float(runs[i]["h"][-1]), i) for i in (1, 2, 3)))
    ys = [v for v, _ in ends]
    for k in range(1, len(ys)):
        if ys[k] - ys[k - 1] < gap:
            ys[k] = ys[k - 1] + gap
    shift = float(np.mean([v for v, _ in ends]) - np.mean(ys))
    ys = [y + shift for y in ys]
    for (h_end, i), y in zip(ends, ys):
        t = runs[i]["t"]
        axR.annotate(f"Agent {i}", xy=(t[-1], h_end),
                     xytext=(t[-1] + tmax * 0.035, y), color=INK,
                     fontsize=FS["dlabel"], fontweight="bold", va="center", zorder=6,
                     arrowprops=dict(arrowstyle="-", color=INK2, lw=1.0,
                                     shrinkA=0, shrinkB=3))
    axR.set_xlabel(r"$t$ [s]", fontsize=FS["label"], fontweight="bold")
    axR.set_ylabel(r"$h$ (barrier value)", fontsize=FS["label"], fontweight="bold")
    axR.set_xlim(0, tmax * 1.16)
    axR.set_ylim(lo, hi)
    axR.tick_params(colors=INK, labelsize=FS["tick"], width=LW["spine"], length=6)
    for lb2 in axR.get_xticklabels() + axR.get_yticklabels():
        lb2.set_fontweight("bold")
    for s in ("top", "right"):
        axR.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        axR.spines[s].set_linewidth(LW["spine"])
    axR.legend(loc="upper left", frameon=False,
               prop={"weight": "bold", "size": FS["legend"]},
               labelcolor=INK, handletextpad=0.6, borderaxespad=0.2)

    fig.tight_layout(w_pad=2.0)
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="ログの時刻プレフィックス 例 141221")
    ap.add_argument("--day", default=None, help="日付フォルダ 既定=今日")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--out", default=None, help="出力png 既定=<日付フォルダ>/<run>_<mode>.png")
    ap.add_argument("--anchors", default=None,
                    help='直接ラベルの位置を上書き。例 \'{"1":[0.7,0.2],"2":[-0.4,0.0],'
                         '"3":[0.1,-0.9]}\'')
    ap.add_argument("--offsets", default=None, help="--anchors と同じ形式でテキストずらし量")
    ap.add_argument("--fontscale", type=float, default=1.0, help="文字サイズ倍率")
    ap.add_argument("--linescale", type=float, default=1.0, help="線幅・マーカー倍率")
    a, _ = ap.parse_known_args()          # ros2 run が渡す --ros-args を無視
    import json
    _scale(a.fontscale, a.linescale)
    if a.anchors:
        ANCHOR.update({int(k): tuple(v) for k, v in json.loads(a.anchors).items()})
    if a.offsets:
        OFFSET.update({int(k): tuple(v) for k, v in json.loads(a.offsets).items()})
    from datetime import datetime
    day = a.day or datetime.now().strftime("%Y-%m-%d")
    runs, mode, d0 = load(a.run, day, a.root)
    out = a.out or os.path.join(d0, f"{a.run}_{mode}.png")
    plot(runs, mode, out)
    print(f"saved: {out}   (mode={mode})")
    for i in (1, 2, 3):
        r = runs[i]
        print(f"  Agent {i}: h min={np.nanmin(r['h']):+.3f}  "
              f"sdf min={np.nanmin(r['sdf']):+.3f}  "
              f"sdf<0 {int((r['sdf'] < 0).sum())} steps  "
              f"slack>0 {int((r['slack'] > 1e-9).sum())} steps")


if __name__ == "__main__":
    main()
