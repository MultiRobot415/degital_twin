# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""障害物mocap橋渡しノード：OptiTrack の剛体を「静止障害物」として /shield/obstacles_state に流す。

pose_bridge が自機の剛体を1個 /tello/pose に流すのと対称の役。こちらは
**自機以外の剛体を複数まとめて**、SHIELD/CBF が読むフラット配列に詰め直す:

    /rigid_bodies (mocap4r2_msgs/RigidBodies … 全剛体の配列)
        │
        ▼  ① obstacle_ids で対象剛体を選ぶ（自機IDは入れない）
           ② Motive座標 → ROS座標(Z-up) へ変換（pose_bridge と同じ規約）
           ③ ID ごとに与えた半径を付けて [x,y,r] を並べる
    /shield/obstacles_state (std_msgs/Float32MultiArray, data=[x1,y1,r1, x2,y2,r2, ...])
        → shield_filter_node が self.obstacles を毎フレーム更新

■ 規約（必ず shield_filter_node・収集/学習と一致させる）
  - 2D 位置 [px,py]（world座標・メートル・Z-up）。半径は物理半径（robot_radius は含めない）
  - 配列の並び順は obstacle_ids の順に固定（IDが見つからない frame でも順序を保つ）

■ ロスト時（オクルージョン）の扱い
  剛体が一時的に見えなくなっても **最後に見えた位置を保持** し続ける
  （静止障害物前提のフェイルセーフ。出力配列から消すと避けられなくなるため）。

■ ID と半径の対応
  obstacle_ids と obstacle_radii を同じ長さで渡す（i番目どうしが対応）。
  ID は Motive の Streaming ID（数値）の文字列。pose_bridge の rigid_body_id と同じ流儀。

使い方:
    ros2 run drone_control obstacle_bridge --ros-args \
        -p obstacle_ids:="[2, 3]" -p obstacle_radii:="[0.30, 0.25]" \
        -p convert_y_up:=false
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32MultiArray
from mocap4r2_msgs.msg import RigidBodies


class ObstacleBridge(Node):
    def __init__(self):
        super().__init__("obstacle_bridge")

        # --- パラメータ ---
        # ID は数値でも文字列でも渡せるよう動的型にして内部で文字列化（pose_bridge と同じ）
        self.declare_parameter(
            "obstacle_ids", ["2"],
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("obstacle_radii", [0.30])
        self.declare_parameter("input_topic", "/rigid_bodies")
        self.declare_parameter("output_topic", "/shield/obstacles_state")
        self.declare_parameter("convert_y_up", False)     # True=Y-up→Z-up変換 / False=素通し
        self.declare_parameter("publish_rate_hz", 20.0)   # 出力レート（最後の値を定期 publish）

        ids = list(self.get_parameter("obstacle_ids").value)
        radii = list(self.get_parameter("obstacle_radii").value)
        self.ids = [str(i) for i in ids]
        self.radii = [float(r) for r in radii]
        if len(self.ids) != len(self.radii):
            raise ValueError(
                f"obstacle_ids と obstacle_radii は同じ長さで渡す: "
                f"ids={self.ids} radii={self.radii}"
            )
        if not self.ids:
            raise ValueError("obstacle_ids が空。少なくとも1つの障害物IDを渡す")

        self.convert_y_up = bool(self.get_parameter("convert_y_up").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        rate = float(self.get_parameter("publish_rate_hz").value)

        # 最後に見えた位置を ID ごとに保持（ロスト時はこれを出し続ける）。未検出は None
        self._last_xy = {i: None for i in self.ids}

        # --- I/O ---
        # ドライバは best_effort で出すので購読側も合わせる（合わないと1本も届かない）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(RigidBodies, in_topic, self._on_rigid_bodies, qos)
        self.pub = self.create_publisher(Float32MultiArray, out_topic, 10)
        # mocap が止まっても最後の位置を出し続けるよう、購読とは独立に定期 publish
        self.create_timer(1.0 / rate, self._publish)

        # 状態監視
        self._got_any = False
        self._matched = {i: False for i in self.ids}
        self.create_timer(2.0, self._watchdog)

        pairs = ", ".join(f"id={i}:r={r:.2f}" for i, r in zip(self.ids, self.radii))
        self.get_logger().info(
            f"obstacle_bridge up. {in_topic} -> {out_topic} | "
            f"obstacles=[{pairs}] convert_y_up={self.convert_y_up} rate={rate}Hz"
        )

    def _on_rigid_bodies(self, msg: RigidBodies):
        self._got_any = True
        for rb in msg.rigidbodies:
            name = rb.rigid_body_name
            if name in self._last_xy:
                x, y = self._to_ros_xy(rb.pose)
                self._last_xy[name] = (x, y)
                if not self._matched[name]:
                    self._matched[name] = True
                    self.get_logger().info(f"障害物 '{name}' を捕捉。位置を流し始める")

    def _to_ros_xy(self, pose):
        """Motive の Pose を ROS(Z-up) の (x, y) に変換して返す（pose_bridge と同じ規約）。

        convert_y_up=False: Motive を Z-up でストリーム → 素通し（x,y をそのまま）。
        convert_y_up=True : Motive 素の Y-up → (x,y,z)->(x,-z,y) の x,y を返す。
        """
        p = pose.position
        if not self.convert_y_up:
            return float(p.x), float(p.y)
        return float(p.x), float(-p.z)

    def _publish(self):
        """ID 順に [x,y,r] を並べた Float32MultiArray を出す。未検出の障害物は飛ばす。"""
        data = []
        for i, r in zip(self.ids, self.radii):
            xy = self._last_xy[i]
            if xy is None:
                continue  # まだ一度も見えていない障害物は出さない（位置不明のため）
            data.extend([xy[0], xy[1], r])
        if not data:
            return  # 出せる障害物がまだ無い
        out = Float32MultiArray()
        out.data = data
        self.pub.publish(out)

    def _watchdog(self):
        if not self._got_any:
            self.get_logger().warn(
                "まだ /rigid_bodies が1本も来ていない。"
                "driver が activate 済みか、QoS(best_effort)・接続(IP/port)を確認"
            )
            return
        missing = [i for i in self.ids if not self._matched[i]]
        if missing:
            self.get_logger().warn(
                f"/rigid_bodies は来ているが obstacle_ids={missing} が見つからない。"
                "Motive の Streaming ID と一致しているか確認"
            )


def main():
    rclpy.init()
    node = ObstacleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
