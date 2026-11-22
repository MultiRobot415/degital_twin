# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""設定ファイルの読み込み・検証のテスト。不正な設定は起動前に止まることを確認する。"""

import copy
import os

import pytest
import yaml

from drone_control import swarm_config

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SWARM = os.path.join(PKG, "config", "swarm.yaml")
FORMATIONS = ["reference_tracking", "leader_follower", "consensus"]


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def fpath(name):
    return os.path.join(PKG, "config", "formation", f"{name}.yaml")


@pytest.fixture
def swarm():
    return swarm_config.load_swarm(SWARM)


def test_shipped_swarm_file(swarm):
    assert swarm.names == ["tello1", "tello2", "tello3"]
    assert swarm.convert_y_up is True
    assert [d.ip for d in swarm.drones] == ["192.168.11.50", "192.168.11.51", "192.168.11.21"]
    assert [d.rigid_body_id for d in swarm.drones] == [1, 2, 3]
    assert swarm.reference_initial == (0.5, 0.0, 1.0)


@pytest.mark.parametrize("name", FORMATIONS)
def test_shipped_formation_files(swarm, name):
    spec = swarm_config.load_formation(fpath(name), swarm)
    assert set(spec.agents) == set(swarm.names)
    # 3 ファイルとも同じ隊形（ref の FORMATION_OFFSETS を ROS に変換したもの）
    assert spec.agents["tello2"].offset == (-0.4, -0.4, 0.0)
    assert spec.agents["tello3"].offset == (-0.4, 0.4, 0.0)


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d["drones"][1].update(name="tello1"), "name が重複"),
    (lambda d: d["drones"][1].update(ip="192.168.11.50"), "ip が重複"),
    (lambda d: d["drones"][1].update(rigid_body_id=1), "rigid_body_id が重複"),
    (lambda d: d["drones"][0].update(ip="192.168.11.500"), "IPv4"),
    (lambda d: d["drones"][0].update(name="1tello"), "英字で始まる"),
    (lambda d: d["mocap"].pop("convert_y_up"), "convert_y_up"),
    (lambda d: d["mocap"].update(convert_y_up="yes"), "明示"),
    (lambda d: d["drones"][0].update(sim_initial_position=[0, 0]), "3要素"),
    (lambda d: d["drones"][0].update(rigid_body_id=True), "整数"),
])
def test_invalid_swarm(mutate, message):
    data = load(SWARM)
    mutate(data)
    with pytest.raises(ValueError, match=message):
        swarm_config.parse_swarm(data)


@pytest.mark.parametrize("base, mutate, message", [
    ("leader_follower", lambda d: d["agents"].pop("tello3"), "不足"),
    ("leader_follower", lambda d: d["agents"].update(tello9={"offset": [0, 0, 0]}), "余分"),
    ("leader_follower", lambda d: d["agents"]["tello2"].update(neighbors={"tello9": 1.0}), "存在しない"),
    ("leader_follower", lambda d: d["agents"]["tello2"].update(neighbors={"tello2": 1.0}), "自分自身"),
    ("leader_follower", lambda d: d["agents"]["tello2"].update(neighbors={"tello1": 0.0}), "> 0.0"),
    # 基準に届かない: tello2 が誰も見ていない
    ("leader_follower", lambda d: d["agents"]["tello2"].update(neighbors={}), "到達できない"),
    # 近傍ゲインが 0 ならフォロワーは実質つながっていない
    ("leader_follower", lambda d: d["control"]["k_nbr"].update(xy=0.0), r"\[xy軸\].*到達できない"),
    ("leader_follower", lambda d: d["control"]["k_nbr"].update(z=0.0), r"\[z軸\].*到達できない"),
    # 誰も基準を受け取らない
    ("consensus", lambda d: d["agents"]["tello1"].update(pinned=False), "基準を受け取る機体が無い"),
    ("reference_tracking", lambda d: d["control"]["k_ref"].update(xy=0.0), "基準を受け取る機体が無い"),
    ("reference_tracking", lambda d: d["control"]["max_cmd"].update(xy=1.5), "<= 1.0"),
    ("reference_tracking", lambda d: d["control"]["max_cmd"].update(z=0.0), "> 0.0"),
    ("reference_tracking", lambda d: d["control"].update(rate_hz=0), "> 0.0"),
    ("reference_tracking", lambda d: d["agents"]["tello1"].update(offset=[0, 0, "a"]), "有限の数値"),
])
def test_invalid_formation(swarm, base, mutate, message):
    data = copy.deepcopy(load(fpath(base)))
    mutate(data)
    with pytest.raises(ValueError, match=message):
        swarm_config.parse_formation(data, swarm)


def test_reachability_through_chain(swarm):
    """tello3 → tello2 → tello1(pinned) の鎖でも基準に届くので OK。"""
    data = load(fpath("leader_follower"))
    data["agents"]["tello3"]["neighbors"] = {"tello2": 1.0}
    spec = swarm_config.parse_formation(data, swarm)
    assert spec.agents["tello3"].neighbors == (("tello2", 1.0),)


def test_formation_file_resolution():
    assert swarm_config.formation_file("/tmp/x.yaml") == "/tmp/x.yaml"
    assert swarm_config.formation_file("rel/dir/x.yml") == "rel/dir/x.yml"
