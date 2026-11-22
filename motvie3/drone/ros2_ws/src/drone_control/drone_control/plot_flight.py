# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""記録した飛行データ（tools/record_flight.sh の rosbag）をグラフにする。

    ros2 run drone_control plot_flight ~/flight_logs/<日時>
        → 同じフォルダに PNG を 3 枚と CSV を保存する
          trajectory.png : 水平面の軌跡（各機・各機の目標・Leader）
          timeseries.png : 各機の x, y, z と目標の時系列（灰色の帯 = enable 中）
          error.png      : 各機の目標からの距離 |p_i − target_i|
          <name>.csv     : 各機の時刻・位置・目標・誤差（Excel 等で開ける）

座標は ROS 規約（x=前, y=左, z=上, world）。目標は各 agent が出す /<name>/formation_target
（formation_agent・consensus_agent のどちらでも出る）。時刻は記録開始からの秒。
"""

import math
import os
import re
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

# 日本語ラベル用のフォント（あるものを使う。無ければ既定フォントで、日本語は表示されない）
for _f in ("Noto Sans CJK JP", "IPAexGothic", "TakaoPGothic", "Droid Sans Fallback"):
    if any(_f == e.name for e in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

from drone_control.frames import yaw_from_quat  # noqa: E402

_POSE_RE = re.compile(r"^/([A-Za-z][A-Za-z0-9_]*)/(pose|formation_target)$")


def read_bag(path):
    """{topic: [(t[s], msg), ...]} を返す。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out, t0 = {}, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t if t0 is None else t0
        out.setdefault(topic, []).append(((t - t0) * 1e-9, deserialize_message(data, types[topic])))
    return out


def series(msgs, getter):
    if not msgs:
        return np.zeros(0), np.zeros((0, 3))
    t = np.array([m[0] for m in msgs])
    v = np.array([getter(m[1]) for m in msgs], dtype=float)
    return t, v


def pos_of_pose(m):
    p = m.pose.position
    return (p.x, p.y, p.z)


def pos_of_odom(m):
    p = m.pose.pose.position
    return (p.x, p.y, p.z)


def enable_intervals(msgs, t_end):
    """[(開始, 終了), ...]（enable=true の区間）。"""
    out, start = [], None
    for t, m in msgs:
        if m.data and start is None:
            start = t
        elif not m.data and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, t_end))
    return out


def main():
    if len(sys.argv) < 2:
        print("使い方: ros2 run drone_control plot_flight <rosbag のフォルダ>")
        sys.exit(2)
    path = os.path.expanduser(sys.argv[1].rstrip("/"))
    data = read_bag(path)
    names = sorted({m.group(1) for t in data for m in [_POSE_RE.match(t)] if m and m.group(2) == "pose"})
    if not names:
        print(f"{path}: /<name>/pose が記録されていない（ROS_DOMAIN_ID が他の端末と同じか確認）")
        sys.exit(1)
    t_end = max(msgs[-1][0] for msgs in data.values())
    en = enable_intervals(data.get("/formation/enable", []), t_end)
    lt, lp = series(data.get("/formation/leader", []), pos_of_odom)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    # ---- 軌跡（水平面）----
    fig, ax = plt.subplots(figsize=(7, 7))
    if len(lt):
        ax.plot(lp[:, 1], lp[:, 0], color="k", lw=1, ls="--", label="leader")
    summary, summary_xy = [], []
    for i, n in enumerate(names):
        c = colors[i % len(colors)]
        t, p = series(data.get(f"/{n}/pose", []), pos_of_pose)
        tt, tp = series(data.get(f"/{n}/formation_target", []), pos_of_pose)
        ax.plot(p[:, 1], p[:, 0], color=c, lw=1.5, label=f"{n}")
        if len(tt):
            ax.plot(tp[:, 1], tp[:, 0], color=c, lw=0.8, ls=":", label=f"{n} target")
        ax.plot(p[0, 1], p[0, 0], "o", color=c, ms=5)
        ax.plot(p[-1, 1], p[-1, 0], "s", color=c, ms=5)
        # 離陸前の機首の向き（矢印。yaw 0 = +x 方向 = 図の上向き）
        q = data[f"/{n}/pose"][0][1].pose.orientation
        yaw0 = yaw_from_quat(q.x, q.y, q.z, q.w)
        ax.annotate("", xy=(p[0, 1] + 0.15 * math.sin(yaw0), p[0, 0] + 0.15 * math.cos(yaw0)),
                    xytext=(p[0, 1], p[0, 0]), arrowprops=dict(arrowstyle="->", color=c, lw=1.5))
        fly = p[:, 2] > 0.5
        if fly.any():
            summary_xy.append(f"{n}: x∈[{p[fly, 0].min():+.2f}, {p[fly, 0].max():+.2f}]  "
                              f"y∈[{p[fly, 1].min():+.2f}, {p[fly, 1].max():+.2f}]  離陸前 yaw {math.degrees(yaw0):+.0f}°")
    # 原点（Motive の原点）
    ax.plot(0.0, 0.0, "k+", ms=14, mew=1.5)
    ax.annotate("原点 (0, 0)", (0.0, 0.0), textcoords="offset points", xytext=(6, -14), fontsize=8)
    ax.text(0.02, 0.02, "飛行中（z > 0.5 m）の範囲\n" + "\n".join(summary_xy) +
            "\n矢印 = 離陸前の機首の向き（yaw 0° = +x = 図の上）",
            transform=ax.transAxes, fontsize=7.5, va="bottom",
            bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.9))
    # x=前 を上、y=左 を左に描く（機体の後ろから見た向き）
    ax.invert_xaxis()
    ax.set_xlabel("y [m]（← 左 / 右 →）")
    ax.set_ylabel("x [m]（前 ↑）")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("水平面の軌跡（○ 開始 / □ 終了）")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "trajectory.png"), dpi=130)
    plt.close(fig)

    # ---- 時系列 ----
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for k, label in enumerate(["x [m]（前）", "y [m]（左）", "z [m]（上）"]):
        a = axes[k]
        for s, e in en:
            a.axvspan(s, e, color="0.9")
        for i, n in enumerate(names):
            c = colors[i % len(colors)]
            t, p = series(data.get(f"/{n}/pose", []), pos_of_pose)
            tt, tp = series(data.get(f"/{n}/formation_target", []), pos_of_pose)
            a.plot(t, p[:, k], color=c, lw=1.2, label=n if k == 0 else None)
            if len(tt):
                a.plot(tt, tp[:, k], color=c, lw=0.8, ls=":")
        a.set_ylabel(label)
        a.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8, ncol=len(names))
    axes[0].set_title("位置（実線）と目標（点線）。灰色 = enable 中")
    axes[-1].set_xlabel("時刻 [s]（記録開始から）")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "timeseries.png"), dpi=130)
    plt.close(fig)

    # ---- 目標からの距離 + CSV ----
    fig, ax = plt.subplots(figsize=(10, 4))
    for s, e in en:
        ax.axvspan(s, e, color="0.9")
    for i, n in enumerate(names):
        t, p = series(data.get(f"/{n}/pose", []), pos_of_pose)
        tt, tp = series(data.get(f"/{n}/formation_target", []), pos_of_pose)
        if len(tt) < 2:
            continue
        inside = (t >= tt[0]) & (t <= tt[-1])
        t, p = t[inside], p[inside]
        tgt = np.column_stack([np.interp(t, tt, tp[:, k]) for k in range(3)])
        err = np.linalg.norm(p - tgt, axis=1)
        ax.plot(t, err, color=colors[i % len(colors)], lw=1.2, label=n)
        np.savetxt(os.path.join(path, f"{n}.csv"), np.column_stack([t, p, tgt, err]), delimiter=",",
                   header="t,x,y,z,target_x,target_y,target_z,error", comments="", fmt="%.4f")
        on = np.zeros_like(t, dtype=bool)
        for s, e in en:
            on |= (t >= s) & (t <= e)
        if on.any():
            summary.append(f"{n}: enable 中の目標からの距離 平均 {err[on].mean():.3f} m / 最大 {err[on].max():.3f} m")
    ax.set_xlabel("時刻 [s]（記録開始から）")
    ax.set_ylabel("|p − target| [m]")
    ax.set_title("目標からの距離。灰色 = enable 中")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(path, "error.png"), dpi=130)
    plt.close(fig)

    print(f"機体: {names} / 記録 {t_end:.1f} 秒 / enable 区間 {[(round(s, 1), round(e, 1)) for s, e in en]}")
    for line in summary:
        print("  " + line)
    print(f"保存: {path}/trajectory.png, timeseries.png, error.png, <name>.csv")


if __name__ == "__main__":
    main()
