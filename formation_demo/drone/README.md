# drone — Tello デジタルツイン（Isaac Sim + ROS2）と 3 機の群フォーメーション

DJI Tello (EDU) の屋内実験アリーナ（3×4m フィールド）を Isaac Sim 上に再現し、
**実機と同じ ROS2 インターフェースで制御をシミュレーションする**デジタルツイン環境。

強化学習ではなく、**制御ループ（モーキャプ → 制御 → 速度指令）の検証**が目的。
そのため Isaac Lab は使わず、Isaac Sim + ROS2 Bridge で構成している。

---

## 2026-10-01 時点の成果物（群フォーメーション・3 機）

当初の 1 機用デジタルツイン（Isaac Sim + ROS2、下の「アーキテクチャ」以降）に加えて、
**Tello 3 機 + OptiTrack による群フォーメーション / ドローンショーを、Sim と実機で同じ制御ノードのまま動かす環境**を作った。
**操作手順は [OPERATION.md](OPERATION.md)**（毎回の準備・キー操作・実機の手順・記録とグラフ・トラブル対応・設定の変え方）。

| 区分 | 成果物 | 状態 |
|---|---|---|
| 実機インフラ | `swarm_real.launch.py`: Motive → mocap4r2 → `pose_bridge`×3 → `/telloN/pose`、`tello_swarm_driver`（3 機を 1 ソケットで扱うドライバ） | 実機で 3 機飛行済み |
| Sim インフラ | `swarm_sim.launch.py`: `swarm_sim`（N 機の軽量シミュレータ。Isaac Sim 不要）。`emulate_mocap:=true` で実機と同じ座標変換経路も通る | 実機と同じトピック・サービス |
| 制御（Sim / 実機 共通） | `formation:=` で切替: `reference_tracking` / `leader_follower` / `consensus`（変位ベース）、`ch3_net1` / `ch3_net2`（**論文 第3章の合意フォーメーション**）、`drone_show`（**正三角形が回転しながら楕円を周回**） | reference_tracking・drone_show は実機飛行済み |
| 操作 | `formation_commander`: 全機離着陸・開始/停止・基準点の移動（1 回 0.1 m / 押しっぱなしで 0.15 m/s）・ゴール指定（RViz / 数値。向きはゆっくり変えて経路を交差させない）・楕円（中心を設定で固定）・隊形の回転 | |
| 記録・解析 | `tools/record_flight.sh`（rosbag を `~/flight_logs/` に保存）、`plot_flight`（軌跡・時系列・誤差の PNG と CSV） | |
| 飛行前チェック | `tools/check_poses.py`（全機の位置・yaw・受信頻度を 1 秒ごとに表示） | |
| 座標系 | Motive(Y-up) / ROS(Z-up) / Tello rc の変換を `frames.py` に集約。変換は入口（pose_bridge）と出口（driver）の 2 か所だけ | 実機で yaw ≈ 0° を確認 |
| テスト | 単体テスト 104 件（座標変換・設定検証・制御則・論文の定理 3.1/系 3.1・ショー）、Sim 結合テスト、偽 Tello 相手のドライバテスト | すべて PASS |

**この環境に合わせて直したもの**: パス（`~/Downloads/degital_twin-main`）、外部パッケージの clone、
tello_driver の CMake（Humble で `rclcpp_components` が必要）、mocap ドライバの CMake（`libNatNet.so` を install）、
Motive PC の IP（**192.168.11.101**。ref のコードの .2 ではない）。

**実機で分かったこと・未対応のこと**
- フィールド前方（x ≳ 1.2 m）では mocap が機体を見失うことがある（Motive は最後の位置を送り続け、制御は古い位置で動き続ける）。
  楕円の中心を (0, 0) に固定して機体が動く範囲を x ∈ [−0.85, 0.85] に収めた（中心固定後の実機飛行はまだ）。
- `vehicle.velocity_scale`（指令 1.0 あたりの速度）は実機で未同定（Sim と同じ 1.0 / 0.6 のまま）。
- 安全機能（飛行範囲の制限・機体間距離・mocap 見失い時の停止・自動着陸）は入れていない。

---

## 1機用デジタルツイン（Isaac Sim + ROS2）— アーキテクチャ

ファイルは「2つの世界」に分かれ、**同じ ROS2 トピックでつながる**。

```
【シミュレーション】
  tello_sim.py (Isaac Sim, venv)         position_controller (ros2_ws)
    剛体Tello ──/tello/pose──▶  ┐                  │
                                ├─ 同じROS2バス ───┤
    剛体Tello ◀──/cmd_vel─────  ┘                  ▼
                                          目標へ飛ぶ指令を計算
```

実機に移すときは**両端だけ差し替える**（中央の制御ノードは共通）:

| | 位置情報の出どころ | /cmd_vel の受け手 |
|---|---|---|
| Sim | `tello_sim.py` が真値poseをpublish | `tello_sim.py`（剛体を駆動） |
| 実機 | OptiTrack(Motive) → mocap4r2ドライバ → `pose_bridge` | tello_driver（実機へUDP） |

実機側の pose は、OptiTrack ドライバが全剛体を `mocap4r2_msgs/RigidBodies` で出すのを、
`pose_bridge` ノードが「対象IDの剛体を1個選び・Z-upに直し・`/tello/pose`(PoseStamped)に詰め直す」。
これで中央の `position_controller` 以降は Sim と全く同じものが動く。

### 物理モデル（現状: B1）
- Tello を**ダイナミック剛体**として扱い、`/cmd_vel`（正規化速度）を剛体の速度として与える
- **重力オフ**（Tello は内部制御で勝手にホバリングする実機の事実に合わせる）
- 衝突オン（地面など当たり判定のある物体に阻まれる）
- 将来 B2: 重力オン＋ホバリング制御 → 推力/トルク で本物の飛行力学へ

---

## ディレクトリ構成

```
drone/
├── sim/                          Isaac Sim 側（venv の python で直接実行。colconの外）
│   ├── tello_sim.py              シミュレータ本体（/cmd_vel購読・真値pose publish）
│   ├── convert_tello_urdf.py     tello.urdf → tello.usd 変換（最初に1回だけ）
│   └── archive/                  開発時の学習用スクリプト（step1〜4 等）
├── usd/
│   ├── env/drone_arena.usd       アリーナ（3×4mフィールド・床・柱・ライト）
│   └── robots/
│       ├── tello.urdf            tello_description から生成（テンプレ変数を埋めた版）
│       └── tello.usd             URDF変換後（剛体・質量・衝突つき）
└── ros2_ws/src/
    ├── drone_control/            ROS2 パッケージ（実機/Sim 共通の制御ロジック）
    │   ├── drone_control/
    │   │   ├── position_controller.py   自動: /tello/pose → 目標へ /cmd_vel（起動時は現在地ホールド）
    │   │   ├── keyboard_teleop.py        手動: キー → /cmd_vel（T=離陸 L=着陸 内蔵）
    │   │   ├── pose_bridge.py            実機: /rigid_bodies → /tello/pose（mocap橋渡し。群では機体数だけ起動）
    │   │   ├── frames.py                 ★座標系・符号の変換（Motive / ROS / Tello rc）をここだけに集約
    │   │   ├── swarm_config.py           群の設定ファイルの読み込み・検証
    │   │   ├── formation_law.py          フォーメーション制御則（ROS非依存の純粋関数）
    │   │   ├── formation_agent.py        群: 変位ベースの制御（1機に1つ。自分＋近傍の pose → /<name>/cmd_vel）
    │   │   ├── consensus_law.py          群: 論文 第3章の合意制御則（ROS非依存）
    │   │   ├── consensus_agent.py        群: 第3章の合意フォーメーション（1機に1つ）
    │   │   ├── show_law.py               群: ドローンショーの目標軌道（ROS非依存）
    │   │   ├── show_agent.py             群: ドローンショー（1機に1つ）
    │   │   ├── leader_trajectory.py      群: 仮想 Leader の軌道（楕円・ゴールへの移動）
    │   │   ├── formation_commander.py    群: キー操作（離着陸・開始/停止・基準点・ゴール・楕円・回転）
    │   │   ├── swarm_sim.py              群: N機の軽量シミュレータ（Isaac Sim 不要）
    │   │   ├── tello_swarm_driver.py     群: 実機 N機を1ソケットで扱うドライバ
    │   │   └── plot_flight.py            群: 記録した rosbag をグラフ・CSV にする
    │   ├── config/swarm.yaml             群: 機体（名前・IP・剛体ID）、基準点・楕円・ショーの回転、速度係数
    │   ├── config/formation/*.yaml       群: 隊形と制御則（reference_tracking / leader_follower / consensus /
    │   │                                        ch3_net1 / ch3_net2 / drone_show）
    │   ├── launch/bringup.launch.py      Sim: controller + RViz を一括起動
    │   ├── launch/bringup_real.launch.py 実機インフラ: mocap+bridge+tello_driver+RViz（飛ばない土台）
    │   ├── launch/swarm_sim.launch.py    群 Sim インフラ: swarm_sim (+ mocap模擬 + pose_bridge) + RViz
    │   ├── launch/swarm_real.launch.py   群 実機インフラ: mocap + pose_bridge×N + tello_swarm_driver + RViz
    │   ├── launch/formation.launch.py    群: 制御ノード × N（formation の law で formation/consensus/show_agent を選ぶ）
    │   ├── rviz/tello.rviz, swarm.rviz   可視化設定
    │   └── test/                         単体テスト（pytest）と test/integration/（Sim・ドライバの結合テスト）
    ├── mocap4ros2_optitrack/             実機mocapドライバ（NatNet同梱）。/rigid_bodies を出す
    │   （※ CMakeLists.txt に libNatNet.so の install を追記済み。params.yaml の server_address = 192.168.11.101）
    ├── ros2_shared/                      tello_driver の依存ライブラリ（外部clone）
    └── tello_ros/                        tello_msgs(型) と tello_driver(実機通訳)。tello_gazebo等は未使用
        （※ ルートの COLCON_IGNORE は外し、tello_msgs / tello_driver のみビルド）
        （※ Humble では tello_driver/CMakeLists.txt の依存に rclcpp_components の追記が必要。追記済み）

drone/tools/
├── tello_set_station.py    Tello を station(子機)モードにする（研究室APに参加させる）
├── check_poses.py          飛行前チェック: 全機の位置・yaw・受信頻度を表示
└── record_flight.sh        飛行データを rosbag で ~/flight_logs/<日時>_<メモ>/ に記録

drone/OPERATION.md          群フォーメーション / ドローンショーの操作マニュアル
```

---

## 初回セットアップ（1機用・Isaac Sim）

```bash
# 1) Tello の URDF → USD 変換（最初に1回だけ・数分かかる）
source ~/env_isaaclab/bin/activate
python ~/Downloads/degital_twin-main/drone/sim/convert_tello_urdf.py
#   ※ 変換後、tello.usd を GUI で開き、ルートprimを「Set as Default Prim」して保存しておくこと

# 2) ROS2 パッケージのビルド
cd ~/Downloads/degital_twin-main/drone/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash
```

---

## 実行（1機用・Isaac Sim、2コマンド運用）

> launch は ROS2 側のノードだけを起動する。Isaac Sim は別env（venv）なので手動で起動する。
> これは実機運用と同じ構図（実機ドローンは別、ROS側だけ launch）。

**① Isaac Sim 側（仮想の機体＋世界）**
```bash
source ~/env_isaaclab/bin/activate
python ~/Downloads/degital_twin-main/drone/sim/tello_sim.py
```

**② ROS2 側（制御 + RViz を一括起動）**
```bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash
ros2 launch drone_control bringup.launch.py
```
→ RViz の **「2D Goal Pose」** でフィールドをクリックすると、その位置へ Tello が飛ぶ。

引数の例:
```bash
ros2 launch drone_control bringup.launch.py goal_x:=-1.0 goal_y:=1.5 goal_z:=0.8 use_rviz:=false
```

### 手動操作（②の代わりに、自分の端末で）
```bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash
ros2 run drone_control keyboard_teleop      # WSAD=平面 / RF=上下 / QE=旋回 / Space=停止
```
※ `position_controller` と `keyboard_teleop` は同時に動かさない（どちらも `/cmd_vel` に書いて競合する）。

---

## ROS2 トピック（1機用）

| トピック | 型 | 向き | 意味 |
|---|---|---|---|
| `/tello/pose` | `geometry_msgs/PoseStamped` | Sim or pose_bridge → | 機体ポーズ（Simは真値 / 実機はmocap由来） |
| `/rigid_bodies` | `mocap4r2_msgs/RigidBodies` | mocapドライバ → pose_bridge | 実機のみ。全剛体まとめ（best_effort） |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop/controller → Sim or tello_driver | 速度指令（正規化 -1〜1） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz → controller | 目標位置（2D Goal Pose） |
| `/flight_data` | `tello_msgs/FlightData` | tello_driver →（実機のみ） | バッテリー・高さ・姿勢など |
| `/tello_action` | `tello_msgs/srv/TelloAction`（**サービス**） | → tello_driver（実機のみ） | 離着陸等（`takeoff`/`land`）。teleopのT/Lキーが呼ぶ |
| TF | `world → tello` | Sim or pose_bridge → | RViz 可視化用 |

---

## 実機運用（1機用: OptiTrack mocap + 実機Tello）

**「インフラ（飛ばない土台）」と「動かすもの」を分離**する構成。launch しても機体は動かず、
動かすのは明示的に1つだけ ros2 run する（/cmd_vel 競合を構造的に防ぐ）。

```
■ レイヤ1: インフラ  bringup_real.launch.py   ← 起動しても“絶対に飛ばない”
    mocap_driver(OptiTrack→/rigid_bodies) + pose_bridge(→/tello/pose)
    + tello_driver(実機通訳) + RViz

■ レイヤ2: 動かすもの  ← 使う時だけ ros2 run（どちらか一方だけ）
    ros2 run drone_control keyboard_teleop       手動（T=離陸 / L=着陸 内蔵）
    ros2 run drone_control position_controller   自動（目標追従・起動時は現在地ホールド）
```

**事前準備（初回のみ）**
- clone 済み: `mocap4ros2_optitrack`（mocapドライバ）, `tello_ros`(tello_driver/tello_msgs), `ros2_shared`
- 依存: `sudo apt install ros-humble-mocap4r2-msgs ros-humble-mocap4r2-control ros-humble-mocap4r2-control-msgs ros-humble-camera-calibration-parsers libasio-dev`
- ビルド: `colcon build`（drone_control / mocap4r2_optitrack_driver / tello_msgs / tello_driver / ros2_shared）
- Motive 側: Tello を Rigid Body 登録 → Streaming ON → その剛体の **Streaming ID** を控える（この研究室は ID=1 が機体1, ID=2 が機体2）。
- mocap driver の `config/mocap4r2_optitrack_driver_params.yaml`：`server_address`(MotivePCのIP=192.168.11.101)/`local_address`(このPC=192.168.11.19)/`connection_type`(この研究室は **Multicast**) を実環境に。
- **座標系**: この研究室の Motive は **Y-up 配信**なので `convert_y_up:=true`（launchの既定値）。Motiveを Z-up にしてあるなら `false`。
- ※ 同梱 NatNet SDK(3.0) は Motive 1.10(NatNet 2.10) のデータも読める。`libNatNet.so` は CMake で install/lib に入れてあるので `source install/setup.bash` だけで実行時に見つかる（手動 LD_LIBRARY_PATH 不要）。
- **Tello を station モードに**（研究室APに子機参加）: `tools/tello_set_station.py` 参照。参加後の Tello のIPを `drone_ip` に渡す。

**① インフラを起動（これだけでは飛ばない）**
```bash
source /opt/ros/humble/setup.bash
cd ~/Downloads/degital_twin-main/drone/ros2_ws && source install/setup.bash
#  機体1(ID=1, IP=192.168.11.50)を使う例:
ros2 launch drone_control bringup_real.launch.py rigid_body_id:=1 drone_ip:=192.168.11.50
```
→ mocap driver の configure→activate は launch が自動。`pose_bridge`→`/tello/pose`、`tello_driver`→`/tello_action`等が立つ。
（`convert_y_up` は既定 true。機体1=.50/機体2=.51, 剛体ID 1/2 に対応）

**② 動かす（別端末で、どちらか一方）**
```bash
ros2 run drone_control keyboard_teleop      # T=離陸 L=着陸 / WSAD・RF・QE=速度 / Space=停止
#  または
ros2 run drone_control position_controller  # RVizの2D Goal Poseで目標指定。起動時は現在地ホールド
```

**検証・トラブル時**
- **座標系の検証**（実機初回は必須）: 機体を手で「前/上/右」に動かし `ros2 topic echo /tello/pose` の x/z/y が期待どおり増えるか確認。ズレてたら `convert_y_up:=true`。
- pose が来ない → `pose_bridge` の警告（mocap未activate / ID不一致 / 接続）を見る。
- 離着陸が効かない → `tello_driver` 起動済みか、`/tello_action` が `ros2 service list` に出るか確認。
- 接続確認: `ros2 topic echo /flight_data`（バッテリー bat、高さ h など）。

## 群フォーメーション（3機・Sim / 実機 共通）— 仕組み

**操作手順は [OPERATION.md](OPERATION.md)。** ここは仕組みと制御則の説明。

1機用の構成（上記）と同じ考え方を N 機に広げたもの。**制御ノードは Sim と実機で同一**で、
差し替えるのは下の「インフラ」だけ。

```
              ┌──────── インフラ（どちらか一方）────────┐
  Sim : swarm_sim.launch.py   swarm_sim ─────────────────────┐
  実機: swarm_real.launch.py  mocap → pose_bridge×N ──────────┤→ /<name>/pose
                              tello_swarm_driver ◀────────────┼── /<name>/cmd_vel
                                                              │
  共通: formation.launch.py   制御ノード × N  ◀── 自分（＋近傍）の pose、Leader / ショー中心
        formation_commander   基準点・仮想 Leader（/formation/leader）・ショー中心（/formation/show）・
                              /formation/enable・全機の離着陸
```

### トピック（群）
| トピック | 型 | 出す側 → 受ける側 | 意味 |
|---|---|---|---|
| `/<name>/pose` | PoseStamped | pose_bridge / swarm_sim → 制御 | 各機の位置・姿勢（world, ROS 座標） |
| `/<name>/cmd_vel` | Twist | 制御 → driver / swarm_sim | 速度指令（正規化 −1〜1, 機体座標） |
| `/<name>/tello_action` | TelloAction（サービス） | commander → driver / swarm_sim | takeoff / land / emergency |
| `/<name>/flight_data` | FlightData | driver / swarm_sim → | バッテリー・高さなど |
| `/<name>/formation_target` | PoseStamped | 制御 → | その機体がいま向かっている点（可視化・記録用） |
| `/<name>/formation_velocity` | TwistStamped | consensus_agent → 近傍 | 第3章の ṙ_j（自分の速度指令, m/s） |
| `/formation/reference` | PoseStamped（latched） | commander → formation_agent | 基準点 r |
| `/formation/leader` | Odometry | commander → consensus_agent | 仮想 Leader の位置・速度・進行方向 |
| `/formation/show` | Odometry | commander → show_agent | ショーの中心・速度・隊形の回転角 |
| `/formation/enable` | Bool（latched） | commander → 制御 | true の間だけ制御ノードが cmd_vel を出す |
| `/formation/goal`, `/goal_pose` | PoseStamped | 端末 / RViz → commander | ゴール指定 |

### 変位ベースの制御（`formation_agent`: reference_tracking / leader_follower / consensus）
各機は **自分・近傍（`neighbors`）・基準（`pinned` の機体だけ）** の情報しか購読しない。
トピックの接続がそのまま機体間の情報グラフになる（`formation_law.py`）:

```
u_i = k_ref·a_i·(r + d_i − p_i) + k_nbr·Σ_{j∈N_i} w_ij·((p_j − d_j) − (p_i − d_i))
      d_i: 隊形オフセット, a_i: pinned なら1, w_ij: 近傍の重み
```

| formation | pinned | neighbors | 意味 |
|---|---|---|---|
| `reference_tracking` | 全機 | なし | 各機が r+d_i を追う（ref の Tモードと同じ構造）。実機の最初はこれ |
| `leader_follower` | tello1 | 2,3 → tello1 | フォロワーはリーダーの実位置を見る |
| `consensus` | tello1 | 完全グラフ | 合意型。neighbors を変えて通信トポロジの影響を調べられる |

- 全機が近傍をたどって pinned 機体に届かない構成は**起動時にエラー**（隊形が一意に決まらないため）。
- 収束の時定数の目安（同梱ゲイン）: reference_tracking / leader_follower 約 2 秒、consensus 約 9 秒（xy）。
  consensus は基準を 1 機しか受け取らないので、全体が揃って動くモードが遅い。

### 論文 第3章の合意フォーメーション（`consensus_agent`: ch3_net1 / ch3_net2）
`ref/article/0_Doctor_Thesis_kuriki.pdf` 第3章（1次系）の制御則（yaml の `law: consensus_ch3`）。

```
u_i = (1/Σ_j a_ij) Σ_j a_ij [ −k(r̂_i − r̂_j) + ṙ_j + (ḋ_i − ḋ_j) ],  r̂ = r − d     … 式 (3.5)
d_i = |d_i| R(ψ_L + θ_i) e_x   （隊形は Leader の進行方向 ψ_L に合わせて回る）          … 式 (3.22)
→ 誤差は e^{−kt} で減衰し、その速さはネットワーク構造によらない                       … 定理3.1・系3.1
```

| 論文 | この実装 |
|---|---|
| Leader（N+1 番） | commander が出す仮想 Leader `/formation/leader` |
| a_i,N+1 / a_ij | yaml の `leader: true` / `neighbors: [...]`（購読するトピックがそのままネットワーク） |
| 近傍の速度 ṙ_j | 近傍が出している速度指令 `/<name>/formation_velocity`（mocap の微分はノイズが大きいため） |
| 隊形情報 d_i | 進行方向 ψ_L だけ全機に直接配信し、各機が d_i, ḋ_i を計算（位置・速度はネットワークどおり） |
| NET1 / NET2（図3.3） | `ch3_net1.yaml` / `ch3_net2.yaml` |
| 表3.2 の配置 | 左右反転（tello2=右 270°, tello3=左 90°）。初期配置のまま表どおりにすると経路が交差するため |
| 楕円軌道（3.4節） | commander の `e`。中心 (0, 0)・a=0.45, b=0.35 m・周期 40 s（`swarm.yaml` の `reference.ellipse`） |
| 論文に無いもの | 速度の上限 `max_speed`、z にも同じ式を適用、進行方向は低速時に保持（3.22 の r0 切替の代わり） |

- 単体テストで定理3.1（e^{−kt} で減衰）・系3.1（NET1 と NET2 で同じ速さ）を数値で確認済み。

### ドローンショー（`show_agent`: drone_show）
各機の目標は時刻だけで決まり、機体同士は通信しない（実際のドローンショーと同じ「決めた軌道を同期して飛ぶ」構造）。

```
p_i*(t) = c(t) + ρ R(φ(t) + θ_i) e_x        c: 中心（= Leader。e で楕円）, φ: 隊形の回転角（o で回転）
u_i     = ṗ_i*(t) − k (p_i − p_i*)            目標の速度をフィードフォワードして追従（第3章の式で相手が目標点1つの場合）
```
- ρ = 0.4 m（正三角形の一辺 0.69 m）、θ = 0°/240°/120°（tello1 前・tello2 右後ろ・tello3 左後ろ。初期配置と経路が交差しない割り当て）。
- 回転 `show.spin_rate` = 0.314 rad/s（20 秒で 1 回転、楕円 1 周 40 秒で 2 回転）。最大速度 約 0.20 m/s。
- Sim での確認: ショー中の目標からの誤差 約 1 cm、三角形の 3 辺 0.703〜0.705 m（理論 0.693 m。応答遅れで一様に約 1.6% 大きい）、
  機体間の最短距離 0.63 m 以上。

### 高さ
どのモードも x・y と同じ式を z にも当てはめ、全機が基準の高さ（`swarm.yaml` の `reference.initial` の z = 1.0 m）へ
P 制御で揃う（mocap の高さを使う）。強さ（誤差 1 m あたりの rc）: reference_tracking 60（ref と同じ）、ch3・drone_show 約 167。

### ⚠ 座標系（最重要）
変換は**境界の 2 か所だけ**で行い、制御ノード・設定ファイルは ROS 規約だけを使う（`frames.py`）。

| | x | y | z | 備考 |
|---|---|---|---|---|
| Motive（Y-up 配信）・ref のコード | 前 | **上** | **右** | pose_bridge が (x, −z, y) に変換（`swarm.yaml` の `mocap.convert_y_up`） |
| **ROS（このパッケージ全体）** | 前 | **左** | 上 | yaw は反時計回りが正 |
| Tello `rc a b c d` | b=前 | a=**右** | c=上 | d=**時計回り**が正 → tello_swarm_driver が符号反転 |

- ref の値を流用するときは必ず変換する。例: ref の 2号機オフセット (−0.4, 0, +0.4)_Motive → (−0.4, **−0.4**, 0)_ROS。
- `pose_bridge` の `convert_y_up` は既定値なし（指定し忘れると起動エラー）。
- 剛体は**機首 +X・Motive の世界軸に揃えて**作成する（yaw 0 の基準になる）。

## 今後の発展

- 中心を (0, 0) に固定した楕円で、ドローンショー・第3章の実機飛行（撮影範囲の端で mocap を見失わないか確認）
- `vehicle.velocity_scale` の実機同定（rc のステップ入力を与えて mocap で速度を測る）→ Sim を実機に近づける
- mocap を見失ったときの扱い（位置が一定時間変わらなければ止める等。安全機能なので入れるかは要判断）
- 第4章（2次系・加速度入力）の合意アルゴリズム、4 機目の追加
- Isaac Sim 側（B2: 重力オン → 推力/トルクで本物の飛行力学）。swarm_sim と同じトピックを出せば差し替えられる
