# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""mocap橋渡しノード：OptiTrack ドライバの出力を /tello/pose に変換する。

実機移行の「左端の差し替え」。Sim では tello_sim.py が真値poseを出していたが、
実機では mocap4r2_optitrack_driver が全剛体をまとめて publish する。型が違うので、
この橋渡しが「Telloの剛体を1個選んで PoseStamped に詰め直す」役をする:

    /rigid_bodies (mocap4r2_msgs/RigidBodies … 全剛体の配列)
        │
        ▼  ① IDで Tello の剛体を1個選ぶ
           ② Motive座標 → ROS座標(Z-up) へ変換
           ③ PoseStamped(frame_id=world) に詰める
    /tello/pose (geometry_msgs/PoseStamped)   → position_controller（無改造）

position_controller は Z-up・世界座標・メートル・yawはZまわり、を前提に読む
（Sim の tello_sim.py と同じ規約）。この橋渡しはその規約に必ず合わせて出す。

■ 座標系（最重要・実機で必ず検証すること）
  Motive のデフォルトは Y-up。ROS は Z-up。`convert_y_up` で吸収する（変換式は frames.py に集約）:
    convert_y_up=false : Motive側のStreaming Up Axis を Z にしてある → 素通し
    convert_y_up=true  : Motiveが素のY-up（この研究室の設定）→ ここで Y-up→Z-up に変換する
  取り違え防止のため convert_y_up に既定値は無い（指定しないと起動時にエラー）。
  どちらかは起動時ログに出る。実機では一度、機体を「前/上/右」に動かして
  /tello/pose の x/y/z が期待通り増えるか確認すること（前→x増, 上→z増, 右→y減）。

■ 剛体の指定
  ドライバは RigidBody.rigid_body_name に「Motive の Streaming ID（数値）の文字列」を
  入れてくる（人間がつけた名前ではない）。`rigid_body_id` をその数値に合わせる。

使い方:
    ros2 run drone_control pose_bridge --ros-args \
        -p rigid_body_id:=1 -p convert_y_up:=true
"""

import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterUninitializedException

from geometry_msgs.msg import PoseStamped, TransformStamped
from mocap4r2_msgs.msg import RigidBodies
from tf2_ros import TransformBroadcaster

from drone_control.frames import motive_yup_to_ros


class PoseBridge(Node):
    def __init__(self):
        super().__init__("pose_bridge")

        # --- パラメータ ---
        # Motive の Streaming ID。CLI で番号(整数)でも渡せるよう動的型にして内部で文字列化
        self.declare_parameter(
            "rigid_body_id", "1",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("input_topic", "/rigid_bodies")
        self.declare_parameter("output_topic", "/tello/pose")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "tello")
        # True=Y-up→Z-up変換 / False=素通し。取り違え防止のため既定値なし（必須）
        self.declare_parameter("convert_y_up", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("publish_tf", True)        # RViz 用に world->body TF も出す

        self.rigid_body_id = str(self.get_parameter("rigid_body_id").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.body_frame = self.get_parameter("body_frame").value
        try:
            self.convert_y_up = bool(self.get_parameter("convert_y_up").value)
        except ParameterUninitializedException:
            raise RuntimeError(
                "convert_y_up を明示すること（true=Motive が Y-up 配信 / false=Z-up 配信）。"
                "例: --ros-args -p convert_y_up:=true") from None
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # --- I/O ---
        # ドライバは best_effort で出すので、購読側も合わせる（合わないと1本も届かない）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(RigidBodies, in_topic, self._on_rigid_bodies, qos)
        self.pose_pub = self.create_publisher(PoseStamped, out_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # 剛体が見つからない/データが来ない状態を気づけるように、定期的に警告
        self._got_any = False
        self._matched_once = False
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f"pose_bridge up. {in_topic} -> {out_topic} | "
            f"rigid_body_id='{self.rigid_body_id}' convert_y_up={self.convert_y_up} "
            f"frame={self.world_frame} tf={self.publish_tf}"
        )

    def _on_rigid_bodies(self, msg: RigidBodies):
        self._got_any = True

        body = None
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == self.rigid_body_id:
                body = rb
                break
        if body is None:
            return  # この frame に対象 ID は無し（watchdog が知らせる）

        if not self._matched_once:
            self._matched_once = True
            self.get_logger().info(f"rigid body '{self.rigid_body_id}' を捕捉。pose を流し始める")

        px, py, pz, qx, qy, qz, qw = self._to_ros_frame(body.pose)

        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.world_frame
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.body_frame
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def _to_ros_frame(self, pose):
        """Motive の Pose を ROS(Z-up) の (x,y,z, qx,qy,qz,qw) に変換して返す。

        convert_y_up=False: Motive を Z-up でストリームしている前提 → 素通し。
        convert_y_up=True : Motive 素のY-up → frames.motive_yup_to_ros
                            (x,y,z) -> (x, -z, y) / quat も同じ並べ替え。
        """
        p, q = pose.position, pose.orientation
        pos, quat = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        if self.convert_y_up:
            pos, quat = motive_yup_to_ros(pos, quat)
        return pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]

    def _watchdog(self):
        if not self._got_any:
            self.get_logger().warn(
                "まだ /rigid_bodies が1本も来ていない。"
                "driver が activate 済みか、QoS(best_effort)・接続(IP/port)を確認"
            )
        elif not self._matched_once:
            self.get_logger().warn(
                f"/rigid_bodies は来ているが rigid_body_id='{self.rigid_body_id}' が見つからない。"
                "Motive の Streaming ID と一致しているか確認"
            )


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C は端末と launch の両方から届くので、後片付けに入ったら 2 回目以降の割り込みは無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
