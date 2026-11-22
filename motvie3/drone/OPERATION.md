# 操作マニュアル — Tello 3 機のフォーメーション / ドローンショー（Sim・実機 共通）

このマニュアルは「群フォーメーション」（`drone_control` パッケージの `swarm_*` / `formation*` 系）の操作手順をまとめたもの。
仕組みや制御則の説明は [README.md](README.md) を参照。

- 座標はすべて ROS 規約: **x = 前, y = 左, z = 上**（Motive の原点が (0, 0)）。yaw は +x 向きが 0°、左回りが正。
- 作業フォルダ: `~/Downloads/degital_twin-main/drone/`（以下 `drone/`）

---

## 1. 初回だけやること

### 1.1 ビルド
```bash
source /opt/ros/humble/setup.bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws
colcon build --packages-up-to drone_control mocap4r2_optitrack_driver tello_driver tello_msgs
```
設定ファイル（`config/*.yaml`）やコードを変えたら、`colcon build --packages-select drone_control` をやり直す。

### 1.2 ネットワーク・機体・Motive

| 項目 | 値 |
|---|---|
| 制御 PC | 192.168.11.19（有線 eno1） |
| Motive PC | **192.168.11.101**（**Motive 3.x / NatNet 4.1** / 120 Hz / Multicast 239.255.42.99:1511）。ping もコマンドポート 1510 も返事が無いが、受信だけなので正常。1.x との違いは [docs/motive3_natnet.md](docs/motive3_natnet.md) |
| Tello | tello1 = 192.168.11.50（剛体 ID 1）, tello2 = .51（ID 2）, tello3 = .21（ID 3） |
| ROS_DOMAIN_ID | **42**（全端末で同じ値。研究室の他の ROS と混線しないため） |

- Tello は station モード（研究室の WiFi に子機として参加）にしておく: `python3 drone/tools/tello_set_station.py "<SSID>" "<パスワード>"`
- Motive: 剛体 ID 1〜3 を登録（**機首を +X に向けて作成**。マーカー配置は機体ごとに左右非対称・別形状）、Up Axis = Y、Streaming ON・Multicast。
- 機体の IP・剛体 ID を変えたら `drone/ros2_ws/src/drone_control/config/swarm.yaml` を直してビルド。

---

## 2. 毎回の準備

### 2.1 端末の準備
端末（ターミナル）は役割ごとに分ける。**どの端末でも最初にこれを実行する**:
```bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=42
```

| 端末 | 役割 | コマンド |
|---|---|---|
| ① | インフラ（Sim か実機のどちらか） | `ros2 launch drone_control swarm_sim.launch.py` または `swarm_real.launch.py` |
| ② | 飛行前チェック（実機のみ） | `python3 ~/Downloads/degital_twin-main/drone/tools/check_poses.py` |
| ③ | 記録 | `bash ~/Downloads/degital_twin-main/drone/tools/record_flight.sh <メモ>` |
| ④ | 制御（隊形・制御則を選ぶ） | `ros2 launch drone_control formation.launch.py formation:=<名前>` |
| ⑤ | 操作（キーボード） | `ros2 run drone_control formation_commander` |

- ①〜④は起動しただけでは**機体は動かない**（離陸は ⑤ の `t`、フォーメーション開始は `g`）。
- **Sim と実機を同じ ROS_DOMAIN_ID で同時に動かさない**（位置データが混ざって暴走する）。
  切り替える前に全端末を Ctrl-C し、`ps -ef | grep "[l]ib/drone_control"` で何も出ないことを確認する。
- ref/mocap_stack のスクリプトや `bringup_real.launch.py`（1 機用）と同時に動かさない（UDP 8889/8890 を取り合う）。

### 2.2 `formation:=` に指定できるもの

| 名前 | 中身 | 使いどころ |
|---|---|---|
| `reference_tracking` | 全機が基準点 + オフセットを追う（ref の T モードと同じ構造） | 実機の最初・1 機だけの試験 |
| `leader_follower` | フォロワーがリーダーの実位置を見る | |
| `consensus` | 変位ベースの合意（tello1 だけ基準を受け取る） | 収束が遅い（時定数 約 9 秒） |
| `ch3_net1` / `ch3_net2` | 論文 第3章の合意フォーメーション（NET1 / NET2）。隊形が Leader の進行方向に合わせて回る | 論文の再現 |
| `drone_show` | 正三角形が中心まわりに回りながら楕円を周回（ルールベース） | ショー |

※ `ch3_*` は近傍の機体の情報が必要なので、1 機だけでは動かない（1 機の試験は `reference_tracking`）。

---

## 3. 操作キー（端末 ⑤ formation_commander）

キーは **端末 ⑤ をクリックしてから** 押す。

| キー | 動作 |
|---|---|
| `t` | 全機 離陸（約 5 秒。高さ約 0.8 m で止まる） |
| `g` | フォーメーション開始（agent が指令を出し始める。高さ 1.0 m に揃う） |
| `h` | フォーメーション停止（全機に 0 指令 → その場でホバリング） |
| `l` | 全機 着陸（**先に `h` を押してから**） |
| `X`（大文字） | 全機 emergency（**モーター即停止 = 落下**。最後の手段） |
| `w` / `s` | 基準点を 前 / 後 へ |
| `a` / `d` | 基準点を 左 / 右 へ |
| `r` / `f` | 基準点を 上 / 下 へ |
| `e` | 楕円軌道の開始 / 停止（中心は `swarm.yaml` の `reference.ellipse.center`。まず開始点へ平行移動し、着いたら自動で周回） |
| `o` | 隊形の回転の開始 / 停止（`drone_show` 用。**`g` で隊形が組めてから押す**） |
| `z` | 初期位置 (0.5, 0, 1.0) へ戻る（ゆっくり移動し、最後に +x を向く） |
| `c` | 楕円・ゴールへの移動を中止（その場で静止） |
| Ctrl-C | commander の終了（終了時に enable=false を出す） |

- `w s a d r f` は **1 回押すと 0.1 m**、**押しっぱなしで 0.15 m/s で動き続け、離すと止まる**。
- それ以外のキーは押しっぱなしでも 1 回だけ効く（0.6 秒以内の同じキーは無視）。`X` だけは常に即時。

### 3.1 ゴールの指定
Leader が ① ゴールの方向へゆっくり向きを変え（0.3 rad/s）② 直進し（0.1 m/s）③ 指定の向きへ回る。
向きをゆっくり変えるので隊形は形を保ったまま回り、機体の経路が交差しない。
- **RViz の 2D Goal Pose**: 上のツールバーの「2D Goal Pose」を押し（または RViz 上で `g` キー）、行き先をクリックして
  到着後の向きの方向へドラッグして離す。高さは今のまま。**1 回指定するとツールが元に戻るので、毎回押し直す**。
- **数値で指定**（別の端末で。xyz を指定、向きを省くと +x）:
  ```bash
  ros2 topic pub --once /formation/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.5, z: 1.2}}}"
  ```

---

## 4. シミュレーターで動かす

```bash
# 端末①
ros2 launch drone_control swarm_sim.launch.py            # RViz が開く。全機地上で待機
# 端末④
ros2 launch drone_control formation.launch.py formation:=drone_show
# 端末⑤
ros2 run drone_control formation_commander
```
- 実機と同じ mocap → pose_bridge の座標変換を通したいときは `swarm_sim.launch.py emulate_mocap:=true`。
- Sim の機体は「指令 1.0 = `swarm.yaml` の `vehicle.velocity_scale` [m/s]・1 次遅れ 0.15 秒」で動く
  （重力・衝突・通信遅延・rc 不感帯は無い）。

### RViz の見方
| 表示 | 意味 |
|---|---|
| 濃い矢印（色ごと） | 各機の現在位置と向き（`/<name>/pose`） |
| 薄い矢印 | 各機が向かっている点（`/<name>/formation_target`） |
| 白い矢印 | 仮想 Leader（`/formation/leader`。向き = 進行方向） |
| マゼンタの矢印 | ドローンショーの中心（`/formation/show`。向き = 隊形の回転角） |
| 黄色の矢印 | 基準点（`/formation/reference`） |

---

## 5. 実機で動かす

### 5.1 飛行前チェック（毎回）
1. 全端末で 2.1 の準備をする。**Sim が動いていないこと**を確認する。
2. Motive の Streaming を ON にしてから、端末①で `ros2 launch drone_control swarm_real.launch.py`。
   - ログに `[natnet_receiver]: 120 Hz, 剛体 ID [1, 2, 3]` と、3 機分の `rigid body 'N' を捕捉` が出ること。
     （Motive 1.x に戻したときは `natnet_version:=2.10` を付ける。旧 mocap4r2 ドライバは `mocap_source:=sdk`）
   - Tello の電源を入れて 20〜30 秒で、3 機分の `[telloN] 状態受信開始（接続）` が出ること。
3. 機体を置く: **3 機とも機首を +X に向けて**、tello1 = 前中央、tello2 = 右後ろ、tello3 = 左後ろ。
4. 端末②の `check_poses.py` で確認:
   - 3 機とも yaw が 0° 付近、受信が約 120 Hz、「止まっている」の警告が無いこと。
   - ラベルと名前が合っているか（ラベル 2 の機体を持ち上げて tello2 の z が増えるか）。
5. バッテリー（`ros2 topic echo /tello1/flight_data --field bat --once`）。ショーなら満充電。
6. 機体が動く範囲がアリーナに収まるか（下の 5.3）。

### 5.2 飛ばす
1. 端末③で記録を開始: `bash ~/Downloads/degital_twin-main/drone/tools/record_flight.sh <メモ>`
2. 端末④で制御を起動: `ros2 launch drone_control formation.launch.py formation:=<名前>`
3. 端末⑤で commander を起動し、キーで操作。

**初めての構成は段階を踏む**: `reference_tracking` で 1 機（他の 2 機は電源オフ）→ 3 機 → `ch3_*` / `drone_show`。

**ドローンショー（`formation:=drone_show`）の順番**:

| 順 | キー | 確認すること |
|---|---|---|
| 1 | `t` | 3 機とも浮くまで待つ |
| 2 | `g` | 正三角形を組み、高さ 1.0 m に揃って止まるまで待つ |
| 3 | `o` | その場で三角形が回る（最初はこれだけで 1〜2 周見る） |
| 4 | `e` | 楕円の開始点へ平行移動（約 6 秒）→ 自動で周回開始。ログに `中心=(+0.00, +0.00)` と出ること |
| 5 | `e` → `o` | 周回と回転を止める |
| 6 | `h` → `l` | 停止してから着陸 |

4. 終わったら端末③を Ctrl-C で止め、グラフを作る（6 章）。

### 5.3 機体が動く範囲
楕円の中心は `swarm.yaml` の `reference.ellipse.center`（既定 (0, 0) = Motive の原点）、半径 a = 0.45 m（x）, b = 0.35 m（y）。
機体はさらに隊形の半径 0.4 m ぶん外側まで動く。

| モード | 機体が動く範囲（中心 (0, 0) のとき） |
|---|---|
| `drone_show`・`ch3_*` で `e` | x ∈ [−0.85, 0.85], y ∈ [−0.75, 0.75] |

- `e` を押すと commander が Leader の範囲をログに出す。
- フィールドの中心が Motive の原点でないときは、フィールド中央に置いた機体の x, y を `check_poses.py` で読み、
  `center` に入れてビルドし直す。
- 撮影範囲の端（2026-10-01 の実機では x ≳ 1.2 m）では mocap が機体を見失いやすい（6.2 参照）。

---

## 6. 記録とグラフ

### 6.1 記録の場所と作り方
- 記録は **自動では残らない**。毎回、端末③で `record_flight.sh` を起動する（Ctrl-C で終了）。
- 保存先: **`/home/shunm/flight_logs/<日時>_<メモ>/`**（`~/flight_logs/` と同じ）。
- グラフと CSV を作る:
  ```bash
  ros2 run drone_control plot_flight ~/flight_logs/<日時>_<メモ>
  xdg-open ~/flight_logs/<日時>_<メモ>/trajectory.png
  ```

| できるファイル | 中身 |
|---|---|
| `<日時>_<メモ>_0.db3`, `metadata.yaml` | 記録データ本体（rosbag） |
| `trajectory.png` | 上から見た軌跡（各機・目標・Leader・原点・離陸前の機首の向き・飛行中の範囲） |
| `timeseries.png` | x, y, z と目標の時間変化（灰色 = フォーメーション中） |
| `error.png` | 目標からの距離の時間変化 |
| `tello1.csv` など | 時刻・位置・目標・誤差（Excel で開ける） |

### 6.2 グラフの読み方の注意
- `timeseries.png` で線が**平らになって急に跳ぶ**区間は、mocap が機体を見失った区間（Motive は最後の位置を送り続ける）。
  その間、制御は古い位置で指令を出し続けている。
- `l` を `h` より先に押すと、着陸後もフォーメーション中として誤差に数えられる。

---

## 7. 困ったとき

| 状況 | すること |
|---|---|
| 変な動き・機体同士が近づく | すぐ `h`（その場で停止）→ 落ち着いたら `l` |
| どうしても止まらない | `X`（大文字）。モーターが止まり落下する |
| commander が落ちた | 別端末で各機に着陸: `ros2 service call /tello1/tello_action tello_msgs/srv/TelloAction "{cmd: land}"`（tello2, tello3 も） |
| `check_poses.py` で「止まっている」 | その機体を mocap が見失っている。すぐ `h` → `l` |
| `[natnet_receiver]: 2 秒間フレームが来ない` | Motive の Streaming が OFF / Transmission Type が Multicast でない / Local Interface が 192.168.11.101 でない。`python3 drone/tools/natnet_dump.py` で生データを確認 |
| `[natnet_receiver]: ... フレーム解析できなかった` | NatNet のバージョン違い（Motive 3.x = `natnet_version:=4.1`、1.x = `2.10`）。Motive を更新したら [docs/motive3_natnet.md](docs/motive3_natnet.md) の手順で確認 |
| `[natnet_receiver]: Motive が見失っている剛体: [N]` | その剛体の位置は流していない（pose が止まる）。すぐ `h` → `l` |
| `rigid_bodies は来ているが rigid_body_id='N' が見つからない` | Motive に ID N の剛体が無い・配信されていない（Motive の剛体の Streaming ID を確認） |
| ログに `not connected :(`（`mocap_source:=sdk` のときだけ） | 旧ドライバは Motive の 1510 の返事が要る。Motive 3.x の PC は返事が無いので既定の `natnet` を使う |
| `入力待ち: ['/tello1/pose']` | 位置データが来ていない（mocap / 剛体 ID / Sim が止まっている） |
| `Not connected, dropping 'takeoff'` | その Tello と通信できていない（電源・WiFi・IP）。電源投入直後は 20〜30 秒待つ |
| `Busy, dropping` | 前のコマンド（離陸など）の応答待ち。少し待ってから押し直す |
| 1 機だけ `'land' ← 'error'` や `Unexpected 'error'` が続く | バッテリー切れのことが多い |
| キーが効かない | 端末⑤をクリックしたか。同じキーを 0.6 秒以内に押していないか |
| RViz のゴールが効かない | 2D Goal Pose ツールを毎回押し直す |
| `Package 'drone_control' not found` | 2.1 の準備（`source install/setup.bash`）を忘れている |

**入れていない安全機能**: 飛行範囲の制限、機体間距離のチェック、mocap を見失ったときの停止、自動着陸。止めるのは人の操作。

---

## 8. 設定の変え方（変えたらビルドし直して、端末④⑤を起動し直す）

すべて `drone/ros2_ws/src/drone_control/config/` の中。

| やりたいこと | ファイル | 項目 |
|---|---|---|
| 楕円の大きさ・周期・中心 | `swarm.yaml` | `reference.ellipse: {a, b, period, center}` |
| ショーの回転の速さ | `swarm.yaml` | `show.spin_rate` [rad/s]（0.314 = 20 秒で 1 回転） |
| 三角形の大きさ・配置 | `formation/drone_show.yaml` | `shape.radius`, 各機の `azimuth_deg`, `dz`（高さをずらす） |
| 基準点の初期位置・キー 1 回の移動量・押しっぱなしの速さ | `swarm.yaml` | `reference.initial`, `step`, `hold_speed` |
| ゴールへの移動の速さ・回転の速さ | `swarm.yaml` | `reference.goto_speed`, `turn_rate` |
| 追従の強さ・速度の上限 | `formation/<名前>.yaml` | `control.k`（または `k_ref` / `k_nbr`）, `max_speed`（または `max_cmd`） |
| 指令 1.0 あたりの速度（実機で同定したら） | `swarm.yaml` | `vehicle.velocity_scale` |
| 機体の IP・剛体 ID・4 機目 | `swarm.yaml` | `drones`（formation 側の `agents` にも同じ名前を足す） |

---

## 9. 動作確認（機体なしでできるテスト）

```bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws/src/drone_control
python3 -m pytest test -q                                         # 単体テスト（座標変換・設定・制御則・ショー）
bash test/integration/run_sim_scenario.sh leader_follower 20 true # Sim 結合（離陸→隊形→移動→停止→着陸）
bash test/integration/run_ch3_scenario.sh ch3_net2 true           # 第3章（静止 Leader → 楕円 → 停止）
bash test/integration/run_driver_check.sh                         # 実機ドライバ（偽 Tello 3 台相手）
```
