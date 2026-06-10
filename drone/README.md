# drone — Tello デジタルツイン（Isaac Sim + ROS2）

DJI Tello (EDU) の屋内実験アリーナ（3×4m フィールド）を Isaac Sim 上に再現し、
**実機と同じ ROS2 インターフェースで制御をシミュレーションする**デジタルツイン環境。

強化学習ではなく、**制御ループ（モーキャプ → 制御 → 速度指令）の検証**が目的。
そのため Isaac Lab は使わず、Isaac Sim + ROS2 Bridge で構成している。

---

## アーキテクチャ

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
    │   │   └── pose_bridge.py            実機: /rigid_bodies → /tello/pose（mocap橋渡し）
    │   ├── launch/bringup.launch.py      Sim: controller + RViz を一括起動
    │   ├── launch/bringup_real.launch.py 実機インフラ: mocap+bridge+tello_driver+RViz（飛ばない土台）
    │   └── rviz/tello.rviz               可視化設定
    ├── mocap4ros2_optitrack/             実機mocapドライバ（NatNet同梱）。/rigid_bodies を出す
    ├── ros2_shared/                      tello_driver の依存ライブラリ（外部clone）
    └── tello_ros/                        tello_msgs(型) と tello_driver(実機通訳)。tello_gazebo等は未使用
        （※ ルートの COLCON_IGNORE は外し、tello_msgs / tello_driver のみビルド）

drone/tools/
└── tello_set_station.py    Tello を station(子機)モードにする（研究室APに参加させる）
```

---

## 初回セットアップ

```bash
# 1) Tello の URDF → USD 変換（最初に1回だけ・数分かかる）
source /home/utsubo/env_isaaclab/bin/activate
python ~/research/drone/sim/convert_tello_urdf.py
#   ※ 変換後、tello.usd を GUI で開き、ルートprimを「Set as Default Prim」して保存しておくこと

# 2) ROS2 パッケージのビルド
cd ~/research/drone/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash
```

---

## 実行（2コマンド運用）

> launch は ROS2 側のノードだけを起動する。Isaac Sim は別env（venv）なので手動で起動する。
> これは実機運用と同じ構図（実機ドローンは別、ROS側だけ launch）。

**① Isaac Sim 側（仮想の機体＋世界）**
```bash
source /home/utsubo/env_isaaclab/bin/activate
python ~/research/drone/sim/tello_sim.py
```

**② ROS2 側（制御 + RViz を一括起動）**
```bash
cd ~/research/drone/ros2_ws && source install/setup.bash
ros2 launch drone_control bringup.launch.py
```
→ RViz の **「2D Goal Pose」** でフィールドをクリックすると、その位置へ Tello が飛ぶ。

引数の例:
```bash
ros2 launch drone_control bringup.launch.py goal_x:=-1.0 goal_y:=1.5 goal_z:=0.8 use_rviz:=false
```

### 手動操作（②の代わりに、自分の端末で）
```bash
cd ~/research/drone/ros2_ws && source install/setup.bash
ros2 run drone_control keyboard_teleop      # WSAD=平面 / RF=上下 / QE=旋回 / Space=停止
```
※ `position_controller` と `keyboard_teleop` は同時に動かさない（どちらも `/cmd_vel` に書いて競合する）。

---

## ROS2 トピック

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

## 実機運用（OptiTrack mocap + 実機Tello）

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
cd ~/research/drone/ros2_ws && source install/setup.bash
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

## 今後の発展

- **B2**: 重力オン → 高さホバリング制御 → 推力/トルク で本物の飛行力学（傾いて進む）
- アリーナのネット/壁に当たり判定を付けて剛体を物理的に閉じ込める
- ✅ 実機の pose 取り込み（OptiTrack → mocap4r2ドライバ → `pose_bridge` → `/tello/pose`）
- ✅ 実機の `/cmd_vel` 受け＋離着陸（`tello_driver`＋`/tello_action`、teleopにT/L統合）
- 残り: 実機での座標系検証・飛行テスト（mocap実機が使えるタイミングで）
