# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""NatNet（OptiTrack Motive のストリーミング）のフレームを解析する純粋関数（ROS 非依存）。

NatNet SDK（libNatNet.so）を使わず、マルチキャストで届くバイト列を直接読む。
SDK は接続時にコマンドポート(1510)の返事でサーバーのバージョンを知るまでフレームを解析しないが、
こちらは受信だけ（ref/mocap_stack の NatNetClient.py と同じ構造）なので Motive への返信は要らない。
その代わりバージョンは設定で与える（Motive 1.x = "2.10", Motive 3.x = "4.1"）。

並びの違いは drone/docs/motive3_natnet.md にまとめてある。要点:
  - NatNet 4.1 以上は各セクションが [個数 int32][バイト数 int32] で始まる（それ未満は個数だけ）
  - 剛体 1 個は 3.0 以上で 38 バイト固定: ID, pos(3f), quat(4f: qx,qy,qz,qw), 平均誤差(f), params(int16)
    2.x は quat の後ろにマーカー配列（数・位置・ID・サイズ）が入る
  - params の bit0 = tracking valid（Motive がその剛体を見失っていると 0）

4.1 以上はセクションのバイト数と解析した長さを照合し、合わなければ NatNetError を投げる
（バージョン設定の取り違えや Motive の更新で並びが変わったのを、黙って変な値を流す前に止める）。
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7

Version = Tuple[int, int]


class NatNetError(ValueError):
    """パケットが想定した並びと合わない。"""


@dataclass(frozen=True)
class RigidBody:
    id: int                                        # Motive の Streaming ID
    position: Tuple[float, float, float]           # Motive の座標そのまま（Up Axis = Y なら x=前, y=上, z=右）[m]
    orientation: Tuple[float, float, float, float]  # (qx, qy, qz, qw)
    mean_error: float                              # マーカーの平均誤差 [m]（2.0 未満は nan）
    tracking_valid: Optional[bool]                 # params bit0。2.6 未満は None（情報なし）


@dataclass(frozen=True)
class Frame:
    frame_number: int
    marker_sets: Tuple[Tuple[str, int], ...]       # (名前, マーカー数)
    n_unlabeled_markers: int
    rigid_bodies: Tuple[RigidBody, ...]
    parsed_bytes: int                              # 解析し終えた位置（4.1 以上はフレーム末尾まで読む）
    # 以下は 4.1 以上でフレーム末尾まで読んだときだけ入る
    timestamp: Optional[float] = None              # Motive 起動からの秒
    stamp_transmit: Optional[int] = None           # 送信時刻（Motive PC の高分解能クロック）


def parse_version(text) -> Version:
    """'4.1' / '2.10' / 4.1 → (4, 1) / (2, 10)。'2.10' を float にすると 2.1 になるので必ず文字列で扱う。"""
    parts = str(text).strip().split(".")
    if len(parts) < 2 or not all(p.isdigit() for p in parts[:2]):
        raise ValueError(f"NatNet のバージョンは 'major.minor' の文字列で指定する（例 '4.1', '2.10'）: {text!r}")
    return int(parts[0]), int(parts[1])


class _Reader:
    """リトルエンディアンのバイト列を先頭から順に読む。足りなければ NatNetError。"""

    def __init__(self, buf: bytes):
        self.buf, self.off = buf, 0

    def _take(self, fmt: str):
        try:
            v = struct.unpack_from(fmt, self.buf, self.off)
        except struct.error:
            raise NatNetError(f"パケットが途中で終わった（位置 {self.off}, 全 {len(self.buf)} バイト）") from None
        self.off += struct.calcsize(fmt)
        return v

    def i32(self) -> int:
        return self._take("<i")[0]

    def i16(self) -> int:
        return self._take("<h")[0]

    def f32(self, n: int = 1):
        v = self._take(f"<{n}f")
        return v if n > 1 else v[0]

    def cstr(self) -> str:
        end = self.buf.find(b"\0", self.off)
        if end < 0:
            raise NatNetError(f"文字列の終端が無い（位置 {self.off}）")
        s = self.buf[self.off:end].decode("utf-8", "replace")
        self.off = end + 1
        return s

    def skip(self, n: int):
        if n < 0 or self.off + n > len(self.buf):
            raise NatNetError(f"{n} バイト読み飛ばせない（位置 {self.off}, 全 {len(self.buf)} バイト）")
        self.off += n


def _section(r: _Reader, ver: Version, name: str):
    """セクション先頭を読む。戻り値: (個数, 終了位置 or None)。4.1 以上はバイト数が付く。"""
    count = r.i32()
    if count < 0:
        raise NatNetError(f"{name}: 個数が負（{count}）。バージョン設定が違う？")
    if ver >= (4, 1):
        size = r.i32()
        if size < 0 or r.off + size > len(r.buf):
            raise NatNetError(f"{name}: バイト数 {size} がパケットに収まらない。バージョン設定が違う？")
        return count, r.off + size
    return count, None


def _end_section(r: _Reader, end: Optional[int], name: str):
    if end is not None and r.off != end:
        raise NatNetError(f"{name}: 解析した長さがセクションのバイト数と合わない（ずれ {r.off - end:+d} バイト）。"
                          "バージョン設定が違う？")


def _rigid_body(r: _Reader, ver: Version) -> RigidBody:
    rb_id = r.i32()
    pos = r.f32(3)
    quat = r.f32(4)
    if ver[0] < 3:
        # 2.x: 剛体の中にマーカー配列（3.0 で削除）
        n = r.i32()
        if n < 0:
            raise NatNetError(f"剛体 {rb_id}: マーカー数が負（{n}）。バージョン設定が違う？")
        r.skip(12 * n)              # 位置
        if ver[0] >= 2:
            r.skip(4 * n)           # ID
            r.skip(4 * n)           # サイズ
    err = r.f32() if ver >= (2, 0) else float("nan")
    valid = bool(r.i16() & 0x01) if ver >= (2, 6) else None
    return RigidBody(rb_id, pos, quat, err, valid)


def parse_frame(payload: bytes, ver: Version) -> Frame:
    """NAT_FRAMEOFDATA の中身（先頭 4 バイトのメッセージ ID・サイズを除いた部分）を解析する。"""
    r = _Reader(payload)
    frame_number = r.i32()

    n, end = _section(r, ver, "marker sets")
    marker_sets = []
    for _ in range(n):
        name = r.cstr()
        nm = r.i32()
        r.skip(12 * nm)
        marker_sets.append((name, nm))
    _end_section(r, end, "marker sets")

    n_unlabeled, end = _section(r, ver, "unlabeled markers")
    r.skip(12 * n_unlabeled)
    _end_section(r, end, "unlabeled markers")

    n, end = _section(r, ver, "rigid bodies")
    bodies = tuple(_rigid_body(r, ver) for _ in range(n))
    _end_section(r, end, "rigid bodies")

    if ver < (4, 1):
        # 4.1 未満はセクションの長さが分からないので、剛体より後ろ（スケルトン等）は読まない
        return Frame(frame_number, tuple(marker_sets), n_unlabeled, bodies, r.off)

    # 4.1 以上: 残りのセクションはバイト数で読み飛ばし、フレーム末尾まで読んで全体の長さを照合する
    for name in ("skeletons", "assets", "labeled markers", "force plates", "devices"):
        _, end = _section(r, ver, name)
        r.off = end
    _timecode, _timecode_sub = r._take("<II")
    (timestamp,) = r._take("<d")
    _mid_exposure, _received, transmit = r._take("<QQQ")
    _precision_sec, _precision_frac = r._take("<II")
    r.i16()                         # フレームの params（記録中など）
    r.i32()                         # end of data
    if r.off != len(payload):
        raise NatNetError(f"フレーム末尾が合わない（解析 {r.off} / 全 {len(payload)} バイト）。バージョン設定が違う？")
    return Frame(frame_number, tuple(marker_sets), n_unlabeled, bodies, r.off, timestamp, transmit)


def parse_message(packet: bytes):
    """UDP パケット → (メッセージ ID, 中身)。"""
    if len(packet) < 4:
        raise NatNetError(f"パケットが短すぎる（{len(packet)} バイト）")
    msg_id, size = struct.unpack_from("<HH", packet)
    return msg_id, packet[4:4 + size]


def pack_message(msg_id: int, body: bytes = b"") -> bytes:
    return struct.pack("<HH", msg_id, len(body)) + body


def parse_serverinfo(payload: bytes):
    """NAT_SERVERINFO → (アプリ名, アプリのバージョン, NatNet のバージョン)。各バージョンは 4 要素タプル。"""
    if len(payload) < 264:
        raise NatNetError("NAT_SERVERINFO が短すぎる")
    name = payload[:256].split(b"\0")[0].decode("utf-8", "replace")
    return name, tuple(payload[256:260]), tuple(payload[260:264])


def parse_modeldef_rigid_bodies(payload: bytes, ver: Version) -> Optional[List[Tuple[int, str, int]]]:
    """NAT_MODELDEF → 剛体の [(Streaming ID, 名前, 親 ID)]。4.1 以上のみ（要素ごとにサイズが付くので他は読み飛ばせる）。"""
    if ver < (4, 1):
        return None
    r = _Reader(payload)
    out = []
    for _ in range(r.i32()):
        dtype = r.i32()
        size = r.i32()
        end = r.off + size
        if dtype == 1:  # 剛体
            name = r.cstr()
            rb_id = r.i32()
            parent = r.i32()
            out.append((rb_id, name, parent))
        r.off = end
    return out
