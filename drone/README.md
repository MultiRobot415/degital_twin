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
    │   │   ├── position_controller.py   自動: /tello/pose → 目標へ /cmd_vel
    │   │   ├── keyboard_teleop.py        手動: キー → /cmd_vel
    │   │   └── pose_bridge.py            実機: /rigid_bodies → /tello/pose（mocap橋渡し）
    │   ├── launch/bringup.launch.py      Sim: controller + RViz を一括起動
    │   ├── launch/tello_real.launch.py   実機: mocapドライバ + bridge + controller を一括起動
    │   └── rviz/tello.rviz               可視化設定
    ├── mocap4ros2_optitrack/             実機mocapドライバ（NatNet同梱）。/rigid_bodies を出す
    └── tello_ros/ (COLCON_IGNORE)        URDF素材。ビルド対象外（将来 tello_msgs が要る時に外す）
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
| `/cmd_vel` | `geometry_msgs/Twist` | → Sim | 速度指令（正規化 -1〜1。実機 tello_ros と同じ） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz → controller | 目標位置（2D Goal Pose） |
| TF | `world → tello` | Sim → | RViz 可視化用 |

---

## 実機運用（OptiTrack mocap）

Sim の代わりに OptiTrack から pose を取り込む。`position_controller` 以降は無改造。

**事前準備（初回のみ）**
- `mocap4ros2_optitrack` を `ros2_ws/src` に clone 済み（`mocap4r2_optitrack_driver`）。
- 依存: `sudo apt install ros-humble-mocap4r2-msgs ros-humble-mocap4r2-control ros-humble-mocap4r2-control-msgs`
- ビルド: `colcon build --packages-select mocap4r2_optitrack_driver drone_control`
- Motive 側: Tello を Rigid Body 登録 → Streaming ON（**Up Axis = Z 推奨**）→ その剛体の **Streaming ID** を控える。
- driver の `config/mocap4r2_optitrack_driver_params.yaml` の `server_address`(MotivePCのIP) と `local_address`(このPCのIP) を実環境に合わせる。

**起動（1コマンド）**
```bash
source /opt/ros/humble/setup.bash
cd ~/research/drone/ros2_ws && source install/setup.bash
ros2 launch drone_control tello_real.launch.py rigid_body_id:=<MotiveのID> convert_y_up:=false
```
→ driver の configure→activate は launch が自動でかける。`pose_bridge` が `/tello/pose` を出し始める。

- `convert_y_up:=true` … Motive を Z-up にできない場合（素のY-up）に橋渡し側で変換。
- **座標系の検証**（実機初回は必須）: 機体を手で「前/上/右」に動かし、`ros2 topic echo /tello/pose` の x/z/y が期待どおり増えるか確認。ズレてたら `convert_y_up` を切り替える。
- pose が来ない時は `pose_bridge` の警告ログ（driver未activate / ID不一致 / 接続）を見る。

`/cmd_vel` の受け手は別途 tello_driver を実機に向けて起動する（このlaunchには含めない）。

## 今後の発展

- **B2**: 重力オン → 高さホバリング制御 → 推力/トルク で本物の飛行力学（傾いて進む）
- アリーナのネット/壁に当たり判定を付けて剛体を物理的に閉じ込める
- `tello_msgs` をビルドして `tello_action`（takeoff/land）に対応
- ✅ 実機の pose 取り込み（OptiTrack → mocap4r2ドライバ → `pose_bridge` → `/tello/pose`）→「実機運用」節へ
- 残り: `/cmd_vel` の受け手を tello_driver に差し替え（実機飛行）
