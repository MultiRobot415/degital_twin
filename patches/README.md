# サブモジュールへのローカル変更（パッチ）

`drone/ros2_ws/src/` の以下2つは **上流本家のリポジトリを submodule で参照している**ため、
こちらから push できない。実験に必要な変更をパッチとして残す。

| パッチ | 対象 | 上流 |
|---|---|---|
| `tello_ros-local-changes.patch` | `drone/ros2_ws/src/tello_ros` | clydemcqueen/tello_ros |
| `mocap4ros2_optitrack-local-changes.patch` | `drone/ros2_ws/src/mocap4ros2_optitrack` | MOCAP4ROS2-Project/mocap4ros2_optitrack |

## 適用方法

```bash
cd drone/ros2_ws/src/tello_ros
git apply ../../../../patches/tello_ros-local-changes.patch
cd ../mocap4ros2_optitrack
git apply ../../../../patches/mocap4ros2_optitrack-local-changes.patch
cd ../../  && colcon build --packages-select tello_driver mocap4r2_optitrack_driver
```

## tello_ros 側の変更の要点

- **`enable_video` パラメータを追加**（既定 true）。`false` にすると `streamon` を送らず
  `VideoSocket` も作らないので、h264 のデコードエラー（`error decoding frame` /
  `concealing N DC ... in P frame`）が出なくなり、映像UDPが WiFi を食わない。
  3機同時運用ではこれを OFF にする（`multiagent.launch.py` の既定は `false`）。
  実装上の注意: 接続判定（takeoff/land のゲート）と keep-alive の条件から映像を外している。
  ここを外し忘れると T キーが弾かれる、または 15 秒で自動着陸する。
- そのほか CMakeLists / ヘッダの変更は本実験より前からあるローカル調整。
