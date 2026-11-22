# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""群制御の設定ファイル（swarm.yaml / formation/*.yaml）の読み込みと検証。

設定は 2 種類に分ける:
    config/swarm.yaml            … 機体（ハード）: 名前・IP・剛体ID・Sim初期位置・基準の初期値
    config/formation/<名前>.yaml … 隊形と制御則: オフセット・基準を受け取る機体・近傍グラフ・ゲイン

すべての座標は ROS 規約（x=前, y=左, z=上, world 座標, メートル）。詳しくは frames.py。

launch・各ノードはどちらもこのモジュールで読み込むので、設定の解釈は1か所に揃う。
不正な設定は起動前に ValueError で止める（飛ばしてから気づくのを防ぐ）。
"""

import ipaddress
import math
import os
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import yaml

Vec3 = Tuple[float, float, float]

PACKAGE = "drone_control"
_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


# =============================================================================
# データ型
# =============================================================================
@dataclass(frozen=True)
class DroneSpec:
    name: str                      # ROS 名前空間になる（/tello1/pose など）
    ip: str                        # station モードでの Tello の IP
    rigid_body_id: int             # Motive の Streaming ID
    sim_initial_position: Vec3     # Sim の初期位置（ROS 座標, 地面なら z=0）
    sim_initial_yaw: float         # Sim の初期ヨー [rad]


@dataclass(frozen=True)
class SwarmSpec:
    world_frame: str
    convert_y_up: bool             # Motive が Y-up 配信なら true（frames.py の ①→②）
    drones: Tuple[DroneSpec, ...]
    reference_initial: Vec3        # フォーメーション基準点の初期値
    reference_step: float          # commander のキー1回あたりの基準移動量 [m]
    # 正規化指令 1.0（rc 100）のときの速度 [m/s]。swarm_sim はこの速度で動き、
    # consensus_agent は m/s の指令を正規化指令に直すのに使う（1か所で定義して Sim と制御を揃える）
    velocity_scale_xy: float = 1.0
    velocity_scale_z: float = 0.6
    # commander の 'e' キーで Leader が動く楕円軌道（論文 3.4 節と同じ考え方）
    ellipse_a: float = 0.45        # x 方向の半径 [m]
    ellipse_b: float = 0.35        # y 方向の半径 [m]
    ellipse_period: float = 40.0   # 1 周の時間 [s]
    ellipse_center: Optional[Tuple[float, float]] = (0.0, 0.0)  # 楕円の中心 (x, y)。None = 今いる点から走り出す
    heading_speed_threshold: float = 0.02  # Leader の速さがこれ未満なら進行方向を更新しない [m/s]
    goto_speed: float = 0.1        # ゴール指定時に Leader が進む速さ [m/s]
    turn_rate: float = 0.3         # ゴール指定時に Leader が向きを変える速さ [rad/s]
    hold_speed: float = 0.15       # キー押しっぱなしで基準が動く速さ [m/s]
    show_spin_rate: float = 0.314  # drone_show: 'o' キーで三角形が回る速さ [rad/s]（0.314 ≈ 20 秒で 1 回転）

    @property
    def names(self) -> List[str]:
        return [d.name for d in self.drones]

    def drone(self, name: str) -> DroneSpec:
        for d in self.drones:
            if d.name == name:
                return d
        raise KeyError(name)


@dataclass(frozen=True)
class AxisGain:
    xy: float
    z: float


@dataclass(frozen=True)
class AgentSpec:
    name: str
    offset: Vec3                                # 隊形オフセット d_i（ROS 座標）
    pinned: bool                                # 基準 r を直接受け取るか（a_i）
    neighbors: Tuple[Tuple[str, float], ...]    # (近傍の名前 j, 重み w_ij>0)


@dataclass(frozen=True)
class ControlSpec:
    rate_hz: float
    k_ref: AxisGain        # 基準追従ゲイン（正規化指令 / m）
    k_nbr: AxisGain        # 近傍結合ゲイン（正規化指令 / m）
    k_yaw: float           # ヨーゲイン（正規化指令 / rad）
    goal_yaw: float        # 全機共通の目標ヨー [rad]（0 = 機首 +x）
    max_cmd_xy: float      # 水平指令のノルム上限（0〜1）
    max_cmd_z: float
    max_cmd_yaw: float


@dataclass(frozen=True)
class FormationSpec:
    description: str
    control: ControlSpec
    agents: Dict[str, AgentSpec]


# =============================================================================
# ファイルの場所
# =============================================================================
def share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(PACKAGE)


def default_swarm_file() -> str:
    return os.path.join(share_dir(), "config", "swarm.yaml")


def formation_file(name_or_path: str) -> str:
    """'leader_follower' のような名前か、yaml へのパスを受け取ってパスを返す。"""
    if name_or_path.endswith((".yaml", ".yml")) or os.sep in name_or_path:
        return name_or_path
    return os.path.join(share_dir(), "config", "formation", f"{name_or_path}.yaml")


# =============================================================================
# 検証ヘルパ
# =============================================================================
def _vec3(value, where: str) -> Vec3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{where}: 3要素のリスト [x, y, z] が必要（ROS座標）: {value!r}")
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f"{where}: 有限の数値が必要: {value!r}")
        out.append(float(v))
    return (out[0], out[1], out[2])


def _number(value, where: str, lo=None, hi=None, lo_open=False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where}: 有限の数値が必要: {value!r}")
    v = float(value)
    if lo is not None and (v < lo or (lo_open and v == lo)):
        raise ValueError(f"{where}: {'>' if lo_open else '>='} {lo} が必要: {v}")
    if hi is not None and v > hi:
        raise ValueError(f"{where}: <= {hi} が必要: {v}")
    return v


def _require(d: dict, key: str, where: str):
    if not isinstance(d, dict) or key not in d:
        raise ValueError(f"{where}: キー '{key}' が必要")
    return d[key]


def _axis_gain(value, where: str) -> AxisGain:
    return AxisGain(xy=_number(_require(value, "xy", where), f"{where}.xy", lo=0.0),
                    z=_number(_require(value, "z", where), f"{where}.z", lo=0.0))


def _load_yaml(path: str) -> dict:
    if not os.path.isfile(path):
        raise ValueError(f"設定ファイルが見つからない: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: トップレベルは mapping である必要がある")
    return data


# =============================================================================
# swarm.yaml
# =============================================================================
def parse_swarm(data: dict, where: str = "swarm.yaml") -> SwarmSpec:
    world_frame = str(data.get("world_frame", "world"))

    mocap = _require(data, "mocap", where)
    convert_y_up = _require(mocap, "convert_y_up", f"{where}.mocap")
    if not isinstance(convert_y_up, bool):
        raise ValueError(f"{where}.mocap.convert_y_up: true/false を明示すること（座標系の取り違え防止）")

    raw_drones = _require(data, "drones", where)
    if not isinstance(raw_drones, list) or not raw_drones:
        raise ValueError(f"{where}.drones: 1機以上のリストが必要")

    drones = []
    for i, d in enumerate(raw_drones):
        w = f"{where}.drones[{i}]"
        name = _require(d, "name", w)
        if not isinstance(name, str) or not _NAME_RE.match(name):
            raise ValueError(f"{w}.name: 英字で始まる英数字/_ の名前が必要（ROS名前空間になる）: {name!r}")
        ip = str(_require(d, "ip", w))
        try:
            ipaddress.IPv4Address(ip)
        except ValueError:
            raise ValueError(f"{w}.ip: IPv4 アドレスが必要: {ip!r}") from None
        rb = _require(d, "rigid_body_id", w)
        if isinstance(rb, bool) or not isinstance(rb, int) or rb < 0:
            raise ValueError(f"{w}.rigid_body_id: 0以上の整数（Motive の Streaming ID）が必要: {rb!r}")
        pos = _vec3(d.get("sim_initial_position", [0.0, 0.0, 0.0]), f"{w}.sim_initial_position")
        yaw = _number(d.get("sim_initial_yaw", 0.0), f"{w}.sim_initial_yaw")
        drones.append(DroneSpec(name, ip, rb, pos, yaw))

    for attr, label in (("name", "name"), ("ip", "ip"), ("rigid_body_id", "rigid_body_id")):
        values = [getattr(d, attr) for d in drones]
        dup = sorted({str(v) for v in values if values.count(v) > 1})
        if dup:
            raise ValueError(f"{where}.drones: {label} が重複している: {dup}")

    ref = _require(data, "reference", where)
    reference_initial = _vec3(_require(ref, "initial", f"{where}.reference"), f"{where}.reference.initial")
    reference_step = _number(ref.get("step", 0.1), f"{where}.reference.step", lo=0.0, lo_open=True)

    vehicle = data.get("vehicle") or {}
    scale = vehicle.get("velocity_scale", {"xy": 1.0, "z": 0.6})
    w = f"{where}.vehicle.velocity_scale"
    vs_xy = _number(_require(scale, "xy", w), f"{w}.xy", lo=0.0, lo_open=True)
    vs_z = _number(_require(scale, "z", w), f"{w}.z", lo=0.0, lo_open=True)

    ell = ref.get("ellipse", {"a": 0.45, "b": 0.35, "period": 40.0})
    we = f"{where}.reference.ellipse"
    ea = _number(_require(ell, "a", we), f"{we}.a", lo=0.0, lo_open=True)
    eb = _number(_require(ell, "b", we), f"{we}.b", lo=0.0, lo_open=True)
    ep = _number(_require(ell, "period", we), f"{we}.period", lo=0.0, lo_open=True)
    if "center" in ell and ell["center"] is None:
        ecenter = None
    else:
        c2 = ell.get("center", [0.0, 0.0])
        if not isinstance(c2, (list, tuple)) or len(c2) != 2:
            raise ValueError(f"{we}.center: [x, y]（ROS 座標）か null が必要: {c2!r}")
        ecenter = (_number(c2[0], f"{we}.center[0]"), _number(c2[1], f"{we}.center[1]"))
    hth = _number(ref.get("heading_speed_threshold", 0.02), f"{where}.reference.heading_speed_threshold",
                  lo=0.0, lo_open=True)

    goto_speed = _number(ref.get("goto_speed", 0.1), f"{where}.reference.goto_speed", lo=0.0, lo_open=True)
    turn_rate = _number(ref.get("turn_rate", 0.3), f"{where}.reference.turn_rate", lo=0.0, lo_open=True)

    hold_speed = _number(ref.get("hold_speed", 0.15), f"{where}.reference.hold_speed", lo=0.0, lo_open=True)
    show = data.get("show") or {}
    spin_rate = _number(show.get("spin_rate", 0.314), f"{where}.show.spin_rate")

    return SwarmSpec(world_frame, convert_y_up, tuple(drones), reference_initial, reference_step,
                     vs_xy, vs_z, ea, eb, ep, ecenter, hth, goto_speed, turn_rate, hold_speed, spin_rate)


def load_swarm(path: str) -> SwarmSpec:
    return parse_swarm(_load_yaml(path), where=os.path.basename(path))


# =============================================================================
# formation/*.yaml
# =============================================================================
def parse_formation(data: dict, swarm: SwarmSpec, where: str = "formation.yaml") -> FormationSpec:
    c = _require(data, "control", where)
    wc = f"{where}.control"
    max_cmd = _require(c, "max_cmd", wc)
    control = ControlSpec(
        rate_hz=_number(_require(c, "rate_hz", wc), f"{wc}.rate_hz", lo=0.0, lo_open=True),
        k_ref=_axis_gain(_require(c, "k_ref", wc), f"{wc}.k_ref"),
        k_nbr=_axis_gain(_require(c, "k_nbr", wc), f"{wc}.k_nbr"),
        k_yaw=_number(_require(c, "k_yaw", wc), f"{wc}.k_yaw", lo=0.0),
        goal_yaw=_number(c.get("goal_yaw", 0.0), f"{wc}.goal_yaw"),
        max_cmd_xy=_number(_require(max_cmd, "xy", f"{wc}.max_cmd"), f"{wc}.max_cmd.xy", 0.0, 1.0, True),
        max_cmd_z=_number(_require(max_cmd, "z", f"{wc}.max_cmd"), f"{wc}.max_cmd.z", 0.0, 1.0, True),
        max_cmd_yaw=_number(_require(max_cmd, "yaw", f"{wc}.max_cmd"), f"{wc}.max_cmd.yaw", 0.0, 1.0, True),
    )

    raw_agents = _require(data, "agents", where)
    if not isinstance(raw_agents, dict):
        raise ValueError(f"{where}.agents: 機体名 → 設定 の mapping が必要")
    expected, given = set(swarm.names), set(raw_agents)
    if expected != given:
        raise ValueError(f"{where}.agents: swarm.yaml の機体と一致しない。"
                         f"不足={sorted(expected - given)} 余分={sorted(given - expected)}")

    agents = {}
    for name in swarm.names:
        a = raw_agents[name]
        wa = f"{where}.agents.{name}"
        offset = _vec3(_require(a, "offset", wa), f"{wa}.offset")
        pinned = a.get("pinned", False)
        if not isinstance(pinned, bool):
            raise ValueError(f"{wa}.pinned: true/false が必要")
        raw_nbrs = a.get("neighbors") or {}
        if not isinstance(raw_nbrs, dict):
            raise ValueError(f"{wa}.neighbors: 近傍名 → 重み の mapping が必要（例 {{tello1: 1.0}}）")
        nbrs = []
        for j, w in raw_nbrs.items():
            if j == name:
                raise ValueError(f"{wa}.neighbors: 自分自身は近傍にできない")
            if j not in expected:
                raise ValueError(f"{wa}.neighbors: 存在しない機体 '{j}'")
            nbrs.append((j, _number(w, f"{wa}.neighbors.{j}", lo=0.0, lo_open=True)))
        agents[name] = AgentSpec(name, offset, pinned, tuple(nbrs))

    spec = FormationSpec(str(data.get("description", "")), control, agents)
    check_well_posed(spec)
    return spec


def load_formation(path: str, swarm: SwarmSpec) -> FormationSpec:
    return parse_formation(_load_yaml(path), swarm, where=os.path.basename(path))


def check_well_posed(spec: FormationSpec) -> None:
    """隊形が一意に決まる（= 全機が基準に収束する）グラフかを軸ごとに確認する。

    単積分器 ṗ_i = u_i・基準 r 一定のもとで、隊形誤差 ξ_i = p_i - d_i - r は
        ξ̇ = -(k_nbr L + k_ref A) ξ     （L: 近傍グラフのラプラシアン, A = diag(a_i)）
    に従う。これが漸近安定 ⇔ 全機が「近傍をたどって」基準を受け取る機体に到達できる
    （有向グラフが pinned 機体を根とする全域森を持つ）。到達できない機体があると、
    その機体群の位置は基準に対して決まらず漂う。

    ゲインが 0 の軸ではその結合は存在しないものとして扱う（xy と z を別々に確認）。
    """
    c = spec.control
    for axis, k_ref, k_nbr in (("xy", c.k_ref.xy, c.k_nbr.xy), ("z", c.k_ref.z, c.k_nbr.z)):
        roots = {n for n, a in spec.agents.items() if a.pinned and k_ref > 0.0}
        if not roots:
            raise ValueError(f"[{axis}軸] 基準を受け取る機体が無い（pinned: true かつ k_ref.{axis} > 0 が1機以上必要）")
        unreachable = []
        for name in spec.agents:
            seen, stack = set(), [name]
            ok = False
            while stack:
                n = stack.pop()
                if n in roots:
                    ok = True
                    break
                if n in seen:
                    continue
                seen.add(n)
                if k_nbr > 0.0:
                    stack.extend(j for j, _ in spec.agents[n].neighbors)
            if not ok:
                unreachable.append(name)
        if unreachable:
            raise ValueError(
                f"[{axis}軸] 基準に到達できない機体がある: {unreachable}。"
                f"pinned にするか、pinned 機体へ近傍をたどって届くよう neighbors を設定すること")


# =============================================================================
# formation/*.yaml のうち law: consensus_ch3（論文 第3章・1次系の合意フォーメーション）
# =============================================================================
CONSENSUS_CH3 = "consensus_ch3"


@dataclass(frozen=True)
class ConsensusAgentSpec:
    name: str
    distance: float                # |d_i|: Leader からの距離 [m]
    azimuth: float                 # θ_i: Leader の進行方向から反時計回りの方位 [rad]
    dz: float                      # 高さ方向のオフセット [m]（論文は水平面のみ。ここでは 0 が基本）
    leader: bool                   # a_i,N+1: Leader の情報（位置・速度）を直接受け取るか
    neighbors: Tuple[str, ...]     # a_ij = 1 となる近傍 j（重みは論文どおりすべて 1）


@dataclass(frozen=True)
class ConsensusSpec:
    description: str
    rate_hz: float
    k: float                       # 収束の速さ [1/s]（定理3.1: 誤差は e^{-kt} で減衰）
    max_speed_xy: float            # 速度指令の上限 [m/s]（水平はノルムで制限）
    max_speed_z: float
    k_yaw: float
    goal_yaw: float
    max_cmd_yaw: float
    rotate_with_leader: bool       # true: 隊形が Leader の進行方向に合わせて回る（式 3.22）
    agents: Dict[str, ConsensusAgentSpec]


def formation_law(path: str) -> str:
    """formation yaml の制御則の種類を返す（'law' キーが無ければ従来の 'displacement'）。"""
    return str(_load_yaml(path).get("law", "displacement"))


def parse_consensus(data: dict, swarm: SwarmSpec, where: str = "formation.yaml") -> ConsensusSpec:
    if data.get("law") != CONSENSUS_CH3:
        raise ValueError(f"{where}: law: {CONSENSUS_CH3} が必要")
    c = _require(data, "control", where)
    wc = f"{where}.control"
    ms = _require(c, "max_speed", wc)
    shape = data.get("shape") or {}
    rotate = shape.get("rotate_with_leader", True)
    if not isinstance(rotate, bool):
        raise ValueError(f"{where}.shape.rotate_with_leader: true/false が必要")

    raw_agents = _require(data, "agents", where)
    if not isinstance(raw_agents, dict):
        raise ValueError(f"{where}.agents: 機体名 → 設定 の mapping が必要")
    expected, given = set(swarm.names), set(raw_agents)
    if expected != given:
        raise ValueError(f"{where}.agents: swarm.yaml の機体と一致しない。"
                         f"不足={sorted(expected - given)} 余分={sorted(given - expected)}")

    agents = {}
    for name in swarm.names:
        a = raw_agents[name]
        wa = f"{where}.agents.{name}"
        leader = a.get("leader", False)
        if not isinstance(leader, bool):
            raise ValueError(f"{wa}.leader: true/false が必要")
        nbrs = a.get("neighbors") or []
        if not isinstance(nbrs, list):
            raise ValueError(f"{wa}.neighbors: 機体名のリストが必要（例 [tello2]）")
        for j in nbrs:
            if j == name:
                raise ValueError(f"{wa}.neighbors: 自分自身は近傍にできない")
            if j not in expected:
                raise ValueError(f"{wa}.neighbors: 存在しない機体 '{j}'")
        if len(set(nbrs)) != len(nbrs):
            raise ValueError(f"{wa}.neighbors: 重複がある")
        if not leader and not nbrs:
            raise ValueError(f"{wa}: Leader とも近傍ともつながっていない（Σa_ij = 0）")
        agents[name] = ConsensusAgentSpec(
            name=name,
            distance=_number(_require(a, "distance", wa), f"{wa}.distance", lo=0.0),
            azimuth=math.radians(_number(_require(a, "azimuth_deg", wa), f"{wa}.azimuth_deg")),
            dz=_number(a.get("dz", 0.0), f"{wa}.dz"),
            leader=leader,
            neighbors=tuple(nbrs),
        )

    unreachable = _unreachable({n: a.neighbors for n, a in agents.items()},
                               {n for n, a in agents.items() if a.leader})
    if unreachable:
        raise ValueError(f"{where}: Leader に到達できない機体がある: {unreachable}"
                         f"（仮定3.1: すべての機体が近傍をたどって Leader の情報を得られること）")

    return ConsensusSpec(
        description=str(data.get("description", "")),
        rate_hz=_number(_require(c, "rate_hz", wc), f"{wc}.rate_hz", lo=0.0, lo_open=True),
        k=_number(_require(c, "k", wc), f"{wc}.k", lo=0.0, lo_open=True),
        max_speed_xy=_number(_require(ms, "xy", f"{wc}.max_speed"), f"{wc}.max_speed.xy", lo=0.0, lo_open=True),
        max_speed_z=_number(_require(ms, "z", f"{wc}.max_speed"), f"{wc}.max_speed.z", lo=0.0, lo_open=True),
        k_yaw=_number(_require(c, "k_yaw", wc), f"{wc}.k_yaw", lo=0.0),
        goal_yaw=_number(c.get("goal_yaw", 0.0), f"{wc}.goal_yaw"),
        max_cmd_yaw=_number(_require(c, "max_cmd_yaw", wc), f"{wc}.max_cmd_yaw", 0.0, 1.0, True),
        rotate_with_leader=rotate,
        agents=agents,
    )


def load_consensus(path: str, swarm: SwarmSpec) -> ConsensusSpec:
    return parse_consensus(_load_yaml(path), swarm, where=os.path.basename(path))


def _unreachable(edges: Dict[str, Tuple[str, ...]], roots) -> List[str]:
    """edges[i] = i が情報を受け取る機体。roots に辿り着けない機体のリストを返す。"""
    out = []
    for name in edges:
        seen, stack, ok = set(), [name], False
        while stack:
            n = stack.pop()
            if n in roots:
                ok = True
                break
            if n in seen:
                continue
            seen.add(n)
            stack.extend(edges[n])
        if not ok:
            out.append(name)
    return out


# =============================================================================
# formation/*.yaml のうち law: drone_show（ルールベースのドローンショー）
# =============================================================================
DRONE_SHOW = "drone_show"


@dataclass(frozen=True)
class ShowSlotSpec:
    name: str
    azimuth: float      # θ_i: 三角形（多角形）の中での位置 [rad]（回転角 φ=0 のとき +x から反時計回り）
    dz: float           # 中心の高さからのオフセット [m]


@dataclass(frozen=True)
class DroneShowSpec:
    """consensus_law.compute_command と同じ名前のフィールドを持つ（同じ関数で指令を計算するため）。"""
    description: str
    rate_hz: float
    k: float            # 目標への追従の速さ [1/s]
    max_speed_xy: float
    max_speed_z: float
    k_yaw: float
    goal_yaw: float
    max_cmd_yaw: float
    radius: float       # ρ: 中心から各機までの距離 [m]
    agents: Dict[str, ShowSlotSpec]

    def min_separation(self) -> float:
        """全機が配置どおりにいるときの最小の機体間距離 [m]（高さの差も含む）。"""
        slots = list(self.agents.values())
        best = float("inf")
        for i in range(len(slots)):
            for j in range(i + 1, len(slots)):
                a, b = slots[i], slots[j]
                dx = self.radius * (math.cos(a.azimuth) - math.cos(b.azimuth))
                dy = self.radius * (math.sin(a.azimuth) - math.sin(b.azimuth))
                best = min(best, math.sqrt(dx * dx + dy * dy + (a.dz - b.dz) ** 2))
        return best


def parse_drone_show(data: dict, swarm: SwarmSpec, where: str = "drone_show.yaml") -> DroneShowSpec:
    if data.get("law") != DRONE_SHOW:
        raise ValueError(f"{where}: law: {DRONE_SHOW} が必要")
    c = _require(data, "control", where)
    wc = f"{where}.control"
    ms = _require(c, "max_speed", wc)
    shape = _require(data, "shape", where)
    radius = _number(_require(shape, "radius", f"{where}.shape"), f"{where}.shape.radius", lo=0.0, lo_open=True)

    raw_agents = _require(data, "agents", where)
    if not isinstance(raw_agents, dict):
        raise ValueError(f"{where}.agents: 機体名 → 設定 の mapping が必要")
    expected, given = set(swarm.names), set(raw_agents)
    if expected != given:
        raise ValueError(f"{where}.agents: swarm.yaml の機体と一致しない。"
                         f"不足={sorted(expected - given)} 余分={sorted(given - expected)}")
    agents = {}
    for name in swarm.names:
        a = raw_agents[name]
        wa = f"{where}.agents.{name}"
        agents[name] = ShowSlotSpec(
            name=name,
            azimuth=math.radians(_number(_require(a, "azimuth_deg", wa), f"{wa}.azimuth_deg")),
            dz=_number(a.get("dz", 0.0), f"{wa}.dz"),
        )
    spec = DroneShowSpec(
        description=str(data.get("description", "")),
        rate_hz=_number(_require(c, "rate_hz", wc), f"{wc}.rate_hz", lo=0.0, lo_open=True),
        k=_number(_require(c, "k", wc), f"{wc}.k", lo=0.0, lo_open=True),
        max_speed_xy=_number(_require(ms, "xy", f"{wc}.max_speed"), f"{wc}.max_speed.xy", lo=0.0, lo_open=True),
        max_speed_z=_number(_require(ms, "z", f"{wc}.max_speed"), f"{wc}.max_speed.z", lo=0.0, lo_open=True),
        k_yaw=_number(_require(c, "k_yaw", wc), f"{wc}.k_yaw", lo=0.0),
        goal_yaw=_number(c.get("goal_yaw", 0.0), f"{wc}.goal_yaw"),
        max_cmd_yaw=_number(_require(c, "max_cmd_yaw", wc), f"{wc}.max_cmd_yaw", 0.0, 1.0, True),
        radius=radius,
        agents=agents,
    )
    if len(agents) > 1 and spec.min_separation() < 1e-6:
        raise ValueError(f"{where}.agents: 同じ位置（azimuth_deg と dz が同じ）の機体がある")
    return spec


def load_drone_show(path: str, swarm: SwarmSpec) -> DroneShowSpec:
    return parse_drone_show(_load_yaml(path), swarm, where=os.path.basename(path))
