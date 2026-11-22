# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""NatNet パケット解析（natnet_protocol.py）のテスト。ここがずれると全機の位置が化ける。"""

import math
import struct

import pytest

from drone_control.natnet_protocol import (NAT_FRAMEOFDATA, NatNetError, parse_frame, parse_message,
                                           parse_modeldef_rigid_bodies, parse_version)

# 2026-10-05 に Motive 3.x（192.168.11.101）から実際に受信したフレーム 1 個（床置きの剛体 ID 2, 1）
MOTIVE3_PACKET = bytes.fromhex(
    "0700c6006915030000000000000000000000000000000000020000004c000000020000001f63f53e5aa0713dbfc889be"
    "e62ee738482ee539c6f52ebafcff7f3f2aea1b39010001000000534aac3df03c183d7a89b03d6e68b5b925cacb394312"
    "f9bae0ff7f3fd2c42439010000000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000aea5362795189a400000000000000000a6d5162e040000003f39172e040000000000000000"
    "000000000000000000")


def i32(v):
    return struct.pack("<i", v)


def quat_yaw_y(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def rb_v3(rb_id, pos, quat, valid=True):
    """NatNet 3.0 以上の剛体 1 個（38 バイト）。"""
    return (i32(rb_id) + struct.pack("<3f", *pos) + struct.pack("<4f", *quat)
            + struct.pack("<f", 0.0005) + struct.pack("<h", 1 if valid else 0))


def rb_v2(rb_id, pos, quat, n_markers=3, valid=True):
    """NatNet 2.x の剛体 1 個（マーカー配列入り）。"""
    return (i32(rb_id) + struct.pack("<3f", *pos) + struct.pack("<4f", *quat)
            + i32(n_markers) + b"\0" * (12 * n_markers) + b"\0" * (4 * n_markers) + b"\0" * (4 * n_markers)
            + struct.pack("<f", 0.0005) + struct.pack("<h", 1 if valid else 0))


def frame_v41(bodies):
    ms = b"tello1\0" + i32(1) + struct.pack("<3f", 1, 2, 3)
    rbs = b"".join(bodies)
    suffix = struct.pack("<IIdQQQIIhi", 0, 0, 12.5, 0, 0, 999, 0, 0, 0, 0)
    return (i32(42) + i32(1) + i32(len(ms)) + ms + i32(0) + i32(0) + i32(len(bodies)) + i32(len(rbs)) + rbs
            + (i32(0) + i32(0)) * 5 + suffix)


def frame_v210(bodies):
    ms = b"tello1\0" + i32(1) + struct.pack("<3f", 1, 2, 3)
    return i32(7) + i32(1) + ms + i32(0) + i32(len(bodies)) + b"".join(bodies) + b"\0" * 32


# -----------------------------------------------------------------------------
def test_parse_version_keeps_two_digit_minor():
    assert parse_version("4.1") == (4, 1)
    assert parse_version("2.10") == (2, 10)   # float にすると 2.1 になってしまう
    assert parse_version("4.1.0.0") == (4, 1)
    with pytest.raises(ValueError):
        parse_version("4")


def test_real_motive3_packet():
    msg_id, payload = parse_message(MOTIVE3_PACKET)
    assert msg_id == NAT_FRAMEOFDATA
    f = parse_frame(payload, (4, 1))
    assert f.parsed_bytes == len(payload) == 198      # フレーム末尾までぴったり読めた
    assert [b.id for b in f.rigid_bodies] == [2, 1]  # 配列の順番は ID 順とは限らない
    by_id = {b.id: b for b in f.rigid_bodies}
    assert by_id[1].position == pytest.approx((0.0842, 0.0372, 0.0862), abs=1e-3)
    assert by_id[2].position == pytest.approx((0.4793, 0.0590, -0.2691), abs=1e-3)
    for b in f.rigid_bodies:
        assert math.sqrt(sum(v * v for v in b.orientation)) == pytest.approx(1.0, abs=1e-4)
        assert b.orientation[3] == pytest.approx(1.0, abs=1e-3)   # 床置き・機首 +X → (0, 0, 0, 1)
        assert b.tracking_valid is True
    assert f.timestamp == pytest.approx(1670.1457, abs=1e-3)


def test_real_motive3_packet_rejected_with_wrong_version():
    _, payload = parse_message(MOTIVE3_PACKET)
    # Motive 1.x 用（2.10）や SDK 3.0 相当（3.0）の並びで読むと、黙って変な値を返さずに止まるか、剛体 0 個になる
    for ver in [(2, 10), (3, 0)]:
        try:
            f = parse_frame(payload, ver)
        except NatNetError:
            continue
        assert len(f.rigid_bodies) == 0


def test_v41_synthetic():
    f = parse_frame(frame_v41([rb_v3(1, (0.5, 1.0, 0.2), quat_yaw_y(90)),
                               rb_v3(3, (0.0, 0.1, -0.4), quat_yaw_y(0), valid=False)]), (4, 1))
    assert f.marker_sets == (("tello1", 1),)
    assert [b.id for b in f.rigid_bodies] == [1, 3]
    assert f.rigid_bodies[0].position == pytest.approx((0.5, 1.0, 0.2))
    assert f.rigid_bodies[0].orientation == pytest.approx(quat_yaw_y(90))
    assert f.rigid_bodies[1].tracking_valid is False
    assert f.stamp_transmit == 999


def test_v41_section_size_mismatch_raises():
    data = bytearray(frame_v41([rb_v3(1, (0, 0, 0), (0, 0, 0, 1))]))
    data += b"\0\0\0\0"   # 末尾にゴミ → フレーム末尾が合わない
    with pytest.raises(NatNetError):
        parse_frame(bytes(data), (4, 1))


def test_v210_motive1_layout():
    f = parse_frame(frame_v210([rb_v2(1, (0.5, 1.0, 0.2), quat_yaw_y(90)),
                                rb_v2(2, (0.0, 0.1, -0.4), quat_yaw_y(0), n_markers=4)]), (2, 10))
    assert [b.id for b in f.rigid_bodies] == [1, 2]
    assert f.rigid_bodies[1].position == pytest.approx((0.0, 0.1, -0.4))
    assert f.rigid_bodies[1].orientation == pytest.approx((0, 0, 0, 1))


def test_truncated_packet_raises():
    _, payload = parse_message(MOTIVE3_PACKET)
    with pytest.raises(NatNetError):
        parse_frame(payload[:60], (4, 1))


def test_modeldef_rigid_body_names():
    body = b"tello1\0" + i32(1) + i32(-1) + b"\0" * 12
    md = i32(2) + i32(0) + i32(4) + b"abcd" + i32(1) + i32(len(body)) + body
    assert parse_modeldef_rigid_bodies(md, (4, 1)) == [(1, "tello1", -1)]
    assert parse_modeldef_rigid_bodies(md, (3, 0)) is None
