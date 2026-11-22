# Motive 1.x → 3.x：NatNet ストリーミングの違いと、自作プログラムでの受け方

2026-10-05、研究室の Motive PC（192.168.11.101）を Motive 1.10 から 3.x に更新した。
このとき、届くデータの並びが変わったこと、旧来の受信コードのどこが壊れるかを、実パケットで確かめた結果をまとめる。
Motive のデータを使うプログラムを新しく書くとき・直すときは、ここを起点にする。

---

## 1. 要点

| | Motive 1.10（旧） | Motive 3.x（今） |
|---|---|---|
| NatNet（データの並びのバージョン） | 2.10 | **4.1 以上**（実パケットの構造から判定。§4） |
| 配信 | マルチキャスト 239.255.42.99:1511、120 Hz | 同じ |
| コマンドポート 1510 の返事 | 返ってきた | **返ってこない**（原因未特定。§2） |
| 剛体 1 個のバイト列 | 可変長（剛体の中にマーカー配列が入る） | **38 バイト固定**（マーカー配列なし） |
| 各セクションの先頭 | `[個数]` | **`[個数][バイト数]`** |
| 座標系 | Y-up（x=前, y=上, z=右）、メートル | 同じ（Up Axis = Y のまま） |
| ref の `NatNetClient.py` | 動く | **剛体 0 個として読む。エラーは出ない**（§5） |
| mocap4r2 ドライバ（NatNet SDK 3.0） | 動く | **`not connected` になり何も出さない**（§5） |

**今の受け方**: `drone_control/natnet_receiver.py`（受信だけで、Motive への返信は不要）。解析は
`drone_control/natnet_protocol.py` で、Motive 1.x にも `natnet_version:="2.10"` で対応する。

---

## 2. 通信の構造

```
Motive PC (192.168.11.101)                         制御 PC (192.168.11.19, eno1)
  ├─ UDP 1511 → マルチキャスト 239.255.42.99 ───▶  受信するだけ（毎フレームの位置・姿勢）     ← データ
  └─ UDP 1510 ◀── 問い合わせ ── / ── 返事 ──▶      接続確認・バージョン・剛体名・フレームレート  ← コマンド
```

- **データ（1511）は一方通行**。Motive が送り出すのを聞いているだけで、こちらから何かを送る必要は無い。
  ref の `NatNetClient.py` はこの方式（PING を 1 回送るが、返事は待たず、解析にも使わない）。
- **コマンド（1510）は問い合わせると返事が来る**仕組み。NatNet SDK（`libNatNet.so`）は接続時にここで
  サーバーのバージョンを受け取り、それが分かるまでフレームを解析しない
  （ライブラリ内に `Error unpacking frame of data. Server version not established.` という文字列がある）。
- Motive 3.x では 1510 に、旧形式の問い合わせを送っても NatNet 4.x 形式で送っても、返事が無い（2026-10-05）。
  データは届いているので、Motive PC の受信側（Windows ファイアウォールは exe ごとに許可するので、新しい Motive は
  許可されていない可能性）か、Motive 3 の設定が怪しい。ただし受信だけのやり方なら 1510 は使わないので、直さなくてよい。
  切り分けたいときは `sudo tcpdump -n -i eno1 udp port 1510` を見ながら `tools/natnet_dump.py` を実行する。

**1510 を使えないことで分からないもの**: Motive の正確なバージョン、剛体の名前（ID ↔ 名前の対応）。
受信側では NatNet のバージョンを設定で与える（Motive 3.x = `4.1`）。

---

## 3. パケットの共通部分

UDP パケット 1 個 = NatNet メッセージ 1 個。すべてリトルエンディアン。

```
[uint16 メッセージ ID][uint16 中身のバイト数][中身 ...]
  ID 7 = NAT_FRAMEOFDATA（フレーム）, 1 = NAT_SERVERINFO, 5 = NAT_MODELDEF
```

---

## 4. フレーム（NAT_FRAMEOFDATA）の並び

### 4.1 並びの比較

`int` = int32, `f` = float32。★ が 1.x と 3.x で違うところ。

| 順 | NatNet 2.10（Motive 1.10） | NatNet 4.1（Motive 3.x） |
|---|---|---|
| 1 | フレーム番号 `int` | 同じ |
| 2 | マーカーセット数 `int` | マーカーセット数 `int` ★ + バイト数 `int` |
|   | 各: 名前（`\0` 終端）, マーカー数 `int`, 位置 `3f`×数 | 同じ |
| 3 | ラベルなしマーカー数 `int`, 位置 `3f`×数 | ★ 個数 `int` + バイト数 `int`, 位置 `3f`×数 |
| 4 | 剛体数 `int` | ★ 剛体数 `int` + バイト数 `int` |
|   | 各剛体（§4.2） | 各剛体（§4.2） |
| 5 | スケルトン, ラベル付きマーカー, フォースプレート, デバイス（長さはどれも中身を読まないと分からない） | ★ スケルトン, **アセット（4.1 で追加）**, ラベル付きマーカー, フォースプレート, デバイス（各 `[個数][バイト数]` 付き） |
| 6 | 末尾: タイムコード, タイムスタンプ, params | ★ 末尾: タイムコード `2×uint32`, タイムスタンプ `double`, カメラ露光中央・データ受信・送信の時刻 `3×uint64`, 高精度タイムスタンプ `2×uint32`, params `int16`, end-of-data `int32` |

4.1 から各セクションに**バイト数**が付いたので、知らないセクションはバイト数ぶん読み飛ばせる。また、
「解析した長さ = バイト数」かどうかで、自分の解析が合っているかを検査できる（`natnet_protocol` はこれで検査する）。

### 4.2 剛体 1 個

| NatNet 2.10（Motive 1.10） | NatNet 3.0 以上（Motive 3.x） |
|---|---|
| ID `int` | ID `int` |
| 位置 `3f` (x, y, z) | 位置 `3f` (x, y, z) |
| 姿勢 `4f` (**qx, qy, qz, qw**) | 姿勢 `4f` (**qx, qy, qz, qw**) — w は最後のまま |
| ★ マーカー数 `int` | — |
| ★ マーカー位置 `3f`×数 | — |
| ★ マーカー ID `int`×数 | — |
| ★ マーカーサイズ `f`×数 | — |
| 平均誤差 `f` [m] | 平均誤差 `f` [m] |
| params `int16`（bit0 = tracking valid） | params `int16`（bit0 = tracking valid） |
| = 38 + 24×マーカー数 バイト | = **38 バイト固定** |

- **ID** は Motive の剛体の Streaming ID（名前ではない）。`swarm.yaml` の `rigid_body_id` がこれ。
- **配列の順番は ID 順とは限らない**（実測で ID 2 → 1 の順に来た）。必ず ID で探す。
- 位置・姿勢は Motive の座標系そのまま。ROS（Z-up）への変換は `frames.motive_yup_to_ros`。

### 4.3 実パケットの読み方（2026-10-05、床置きの剛体 2 個）

```
07 00 c6 00                  メッセージ ID = 7（フレーム）, 中身 198 バイト
69 15 03 00                  フレーム番号
00000000 00000000            マーカーセット: 0 個, 0 バイト
00000000 00000000            ラベルなしマーカー: 0 個, 0 バイト
02000000 4c000000            剛体: 2 個, 76 バイト（= 38 × 2）
  02000000                     ID = 2
  1f63f53e 5aa0713d bfc889be   位置 (0.479, 0.059, -0.269)
  e62ee738 482ee539 c6f52eba fcff7f3f   姿勢 (≈0, ≈0, ≈0, 1.0)
  2aea1b39                     平均誤差 0.00015 m
  0100                         params = 1（tracking valid）
  01000000 ...                 ID = 1 ...
00000000 00000000 ×5         スケルトン / アセット / ラベル付きマーカー / フォースプレート / デバイス: 各 0
末尾 54 バイト               タイムコード, タイムスタンプ 1670.15 s, 時刻 3 つ, 高精度時刻, params, end-of-data
```

198 バイトがぴったり読み切れるので、4.1 以上の並びで間違いない（このパケットは `test/test_natnet_protocol.py` に入れてある）。
カメラ露光中央の時刻は 0 で来ていた（遅延の計算には使えない）。マーカーセットも 0 個（Motive 3 の既定で配信していない）。

---

## 5. 既存コードへの影響

### ref/mocap_stack の `NatNetClient.py`（自前の解析、2.x 形式）
剛体の数を数える位置で 4 バイトずれる。そのため Motive 3.x のパケットは「剛体 0 個」と読まれ、**エラーを出さずに何も呼ばれない**:

```
実際:     [frame][marker sets 個数=0][バイト数=0][unlabeled 個数=0][バイト数=0][剛体 個数=2][バイト数=76]
ref の読み: [frame][marker sets 個数=0][unlabeled 個数 ←0 ][剛体 個数 ←0 ] → 終了
```
実パケットで確認済み（`rigidBodyListener` は 0 回、`rigidBodyCount = 0`）。剛体まで読めたとしても、
剛体の中のマーカー配列（2.x のみ）を読もうとしてずれる。また、バージョンは `(3,0,0,0)` 固定で、サーバーから更新されない。
→ 使うなら `drone_control/natnet_protocol.py` を使う（§6）か、NatNet SDK 4.x に付属する公式 Python サンプルに差し替える。

### mocap4r2_optitrack_driver（NatNet SDK 3.0）
1510 の返事が無いので `... not connected :(` になり、`/rigid_bodies` が 1 本も出ない（実測）。
`swarm_real.launch.py` / `bringup_real.launch.py` では `mocap_source:=sdk` を指定したときだけ起動する。
既定では `natnet_receiver` を使う。

### natnet_receiver（今の既定）
`/rigid_bodies`（`mocap4r2_msgs/RigidBodies`）を mocap4r2 ドライバと同じ形で出す（`rigid_body_name` = ID の文字列）。
`pose_bridge` 以降は変更なし。違うのは次の点:
- **見失った剛体（tracking valid = 0）は出さない**（`drop_untracked:=true`）。
  見失うと、その機体の `/telloN/pose` が止まり、`check_poses.py` に「止まっている」と出る。
  Motive 1.10 は見失っても最後の位置を送り続けていた。Motive 3.x が見失ったときに何を送るかは未確認（§7）。
- `header.stamp` は受信した時刻（Motive 側の遅延分は引いていない）。

---

## 6. 自作プログラムで使う

ROS 無しで使える（`natnet_protocol.py` は標準ライブラリだけで書いてある）:

```python
import socket, sys
sys.path.insert(0, "<drone>/ros2_ws/src/drone_control")
from drone_control import natnet_protocol as nn

VER = nn.parse_version("4.1")          # Motive 3.x。Motive 1.x は "2.10"（float の 2.1 にしないこと）
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 1511))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton("239.255.42.99") + socket.inet_aton("192.168.11.19"))
while True:
    msg_id, payload = nn.parse_message(sock.recvfrom(65535)[0])
    if msg_id != nn.NAT_FRAMEOFDATA:
        continue
    try:
        frame = nn.parse_frame(payload, VER)    # 並びが合わなければ NatNetError（黙って変な値は返さない）
    except nn.NatNetError as e:
        print("解析失敗:", e); continue
    for rb in frame.rigid_bodies:               # 順番は ID 順とは限らない
        if rb.tracking_valid is False:          # 見失い。位置を信用しない
            continue
        print(rb.id, rb.position, rb.orientation)   # Motive 座標（Y-up）, (qx, qy, qz, qw)
```

- 同じ PC で複数のプログラムが受けてよい（`SO_REUSEADDR` を付ければ、マルチキャストは全員に届く）。
- ROS では `/rigid_bodies`（Motive 座標）か `/telloN/pose`（ROS 座標）を購読すれば十分。
- C++ で NatNet SDK を使う場合は **SDK 4.x** を使い、かつ Motive PC の 1510 の返事が届くようにする必要がある。

---

## 7. 未確認のこと

| 項目 | 状態 | 確かめ方 |
|---|---|---|
| Motive の正確なバージョン（3.0 / 3.1 / …）、NatNet 4.1 か 4.2 以上か | 1510 の返事が無いので不明。剛体まわりの並びは 4.1 以上で共通なので、今の受信には影響しない | Motive の Help → About |
| 1510 に返事が無い原因 | Motive PC 側（ファイアウォール・設定）の可能性が高い | 上の tcpdump の方法 |
| 見失ったときに Motive 3.x が送るもの（最後の値 / 0 / 配列から消える） | 未確認。受信側は tracking valid = 0 を捨てるので、どれでも安全側 | 剛体を布で隠して `tools/natnet_dump.py` を見る |
| ID 3（tello3）の剛体 | 2026-10-05 時点では配信されていない（ID 1, 2 のみ） | Motive で剛体 3 を登録して Streaming ID を確認 |

## 8. Motive を更新・設定変更したときの確認手順

1. `python3 drone/tools/natnet_dump.py --hex` を実行して、次を確認する。
   - `受信 120 Hz` と出る
   - 全剛体の `|q| = 1.0000`
   - `解析失敗` が出ない
   - 「〇バイト中〇バイトまで解析」の 2 つが同じ（4.1 以上の場合）
2. 剛体の ID が合っているか：機体を 1 機ずつ持ち上げて、その ID の y（上）だけが増えるか。
3. 座標系：前・上・右に動かして、生の値で x・y・z がそれぞれ増えるか。`check_poses.py` では x 増・z 増・y 減になるか。
4. 機首を +X に向けて床に置いたとき、q ≈ (0, 0, 0, 1) で、`check_poses.py` の yaw ≈ 0° になるか。
5. 並びが変わっていたら、`natnet_protocol.py` を直す。実パケットを `test_natnet_protocol.py` に足して、テストを通す。
