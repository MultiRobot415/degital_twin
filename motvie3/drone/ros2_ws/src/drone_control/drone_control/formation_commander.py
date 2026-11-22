# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""フォーメーションの操作卓（キーボード）。Sim でも実機でも同じものを使う。

このノードが持つのは「基準点 r（= 仮想 Leader）」と「enable」だけで、機体の cmd_vel は一切出さない
（cmd_vel を出すのは各機の formation_agent / consensus_agent）。

    配信   /formation/reference (PoseStamped, world, latched)  … 基準点 r（formation_agent 用）
           /formation/leader    (Odometry, world, 20Hz)         … Leader の位置・速度・進行方向（consensus_agent 用）
           /formation/show      (Odometry, world, 20Hz)         … ショーの中心・速度・隊形の回転角（show_agent 用）
           /formation/enable    (Bool, latched)                 … true の間だけ agent が cmd_vel を出す
    呼出   /<name>/tello_action                                 … 全機の離陸・着陸・emergency
    購読   /goal_pose (RViz の 2D Goal Pose)                    … ゴールの xy と到着後の向きをクリックで指定
           /formation/goal (PoseStamped, world)                 … ゴールの xyz と到着後の向きを数値で指定

ゴールを指定すると Leader は ① その場でゴールの方向を向き ② 直進し ③ 指定の向きへ回る
（向きはゆっくり変えるので、隊形は形を保ったまま回る = 機体の経路が交差しない）。
数値で指定する例（向きを省くと +x）:
    ros2 topic pub --once /formation/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.5, z: 1.2}}}"

キー（ROS 座標: x=前, y=左, z=上）:
    t : 全機 離陸              l : 全機 着陸
    g : フォーメーション開始    h : フォーメーション停止（各機に 0 指令 → ホバリング）
    w / s : 基準 +x / -x（前 / 後）   a / d : 基準 +y / -y（左 / 右）   r / f : 基準 +z / -z（上 / 下）
    e : 楕円軌道の開始 / 停止（論文 3.4 節）。swarm.yaml の reference.ellipse.center を中心に回る。
        押すとまず楕円の開始点へ向きを保ったまま平行移動し、着いたら自動で周回を始める
        （center: null なら従来どおり、今いる点から今の進行方向に沿って走り出す）
    z : 初期位置へ戻る（ゴール指定と同じ動きで移動し、最後に +x を向く）
    c : 楕円・ゴールへの移動を中止（その場で静止）
    o : 隊形の回転の開始 / 停止（drone_show 用。swarm.yaml の show.spin_rate で中心まわりに回る）
    X（大文字）: 全機 emergency（モーター即停止 = 落下する）
    Ctrl-C : 終了（終了時に enable=false を出す）

※ 自分の端末で起動すること（キー入力を読むので ros2 launch には入れない）。
※ w/s/a/d/r/f: 1回押すと reference.step（0.1m）だけ動く。押しっぱなしにすると reference.hold_speed
  （0.15 m/s）で滑らかに動き続け、離すと止まる。隊形の向きは変えない（急に回らない）。
  それ以外のキー（t, l, g, h, e, z, c）は押しっぱなしでも1回だけ効く。X（emergency）は常に即時に効く。

実行:
    ros2 run drone_control formation_commander
"""

import math
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction

from drone_control import swarm_config
from drone_control.formation_agent import ENABLE_TOPIC, LATCHED_QOS, REFERENCE_TOPIC
from drone_control.frames import quat_from_yaw, yaw_from_quat
from drone_control.leader_trajectory import Ellipse, GoTo, ellipse_start_point

LEADER_TOPIC = "/formation/leader"
SHOW_TOPIC = "/formation/show"
LEADER_RATE_HZ = 20.0

HELP = __doc__
# 同じキーの入力が前回からこの時間以内なら「押しっぱなしのキーリピート」とみなす。
# OS のキーリピートは押してから約 0.5 秒後に始まり、以後 30 回/秒ほど来るので、0.6 秒あれば区別できる。
# （以前は d の押しっぱなしで基準が 0.03 秒ごとに 0.1m ずつ、3m 以上飛んだ。今は一定速度で動く）
KEY_REPEAT_GAP = 0.6
# 押しっぱなし移動中、キーリピートがこの時間来なければ「離した」とみなして止める
HOLD_RELEASE = 0.15
MOVE_KEYS = {  # キー → (軸, 符号)
    "w": (0, +1), "s": (0, -1),
    "a": (1, +1), "d": (1, -1),
    "r": (2, +1), "f": (2, -1),
}
RC_NAMES = {TelloAction.Response.OK: "OK", TelloAction.Response.ERROR_NOT_CONNECTED: "NOT_CONNECTED",
            TelloAction.Response.ERROR_BUSY: "BUSY"}


class FormationCommander(Node):
    def __init__(self):
        super().__init__("formation_commander")
        self.declare_parameter("swarm_file", swarm_config.default_swarm_file())
        self.swarm = swarm_config.load_swarm(self.get_parameter("swarm_file").value)
        self.world_frame = self.swarm.world_frame
        self.reference = list(self.swarm.reference_initial)
        self.velocity = [0.0, 0.0, 0.0]   # Leader の速度（動いているときだけ非ゼロ）
        self.heading = 0.0                # Leader の進行方向 ψ（初期値 = +x）
        self.heading_rate = 0.0
        self.motion = None                # (Ellipse か GoTo, 開始時刻[s])
        self.pending_ellipse = False      # 楕円の開始点へ移動中（着いたら楕円を始める）
        self.hold = None                  # 押しっぱなし移動 (軸, 符号, 最後のキーリピート時刻[monotonic])
        self.spin_phase = 0.0             # drone_show: 隊形の回転角 φ [rad]
        self.spinning = False
        self._last_tick = time.monotonic()
        self.enabled = False

        self.ref_pub = self.create_publisher(PoseStamped, REFERENCE_TOPIC, LATCHED_QOS)
        self.leader_pub = self.create_publisher(Odometry, LEADER_TOPIC, 10)
        self.show_pub = self.create_publisher(Odometry, SHOW_TOPIC, 10)
        self.create_timer(1.0 / LEADER_RATE_HZ, self._update_leader)
        self.enable_pub = self.create_publisher(Bool, ENABLE_TOPIC, LATCHED_QOS)
        self.action_clients = {n: self.create_client(TelloAction, f"/{n}/tello_action") for n in self.swarm.names}
        self.create_subscription(PoseStamped, "/goal_pose", lambda m: self._on_goal(m, "/goal_pose", True), 10)
        self.create_subscription(PoseStamped, "/formation/goal",
                                 lambda m: self._on_goal(m, "/formation/goal", False), 10)

        self.publish_enable(False)
        self.publish_reference()

    # ------------------------------------------------------------------ 基準・enable
    def publish_reference(self, log: bool = True):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.reference
        msg.pose.orientation.w = 1.0
        self.ref_pub.publish(msg)
        if log:
            r = self.reference
            self.get_logger().info(f"基準 r = ({r[0]:+.2f}, {r[1]:+.2f}, {r[2]:+.2f}) [ROS: x前 y左 z上]")

    # ------------------------------------------------------------------ 仮想 Leader
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def toggle_ellipse(self):
        if self.pending_ellipse or isinstance(self.motion and self.motion[0], Ellipse):
            self.stop_motion()
            self.get_logger().info("楕円軌道 停止（その場で静止）")
            return
        sw = self.swarm
        self.stop_motion()
        if sw.ellipse_center is not None:
            # 中心が決まっている: 今の向きのまま楕円に接して走り出せる点へ、まず平行移動する
            center = (sw.ellipse_center[0], sw.ellipse_center[1], self.reference[2])
            start = ellipse_start_point(center, self.heading, sw.ellipse_a, sw.ellipse_b)
            if math.dist(start, self.reference) > 0.01:
                g = GoTo(tuple(self.reference), self.heading, start, sw.goto_speed, sw.turn_rate, turn=False)
                self.motion = (g, self._now())
                self.pending_ellipse = True
                self.get_logger().info(
                    f"楕円の開始点 ({start[0]:+.2f}, {start[1]:+.2f}) へ平行移動中（約 {g.duration:.1f} 秒）→ "
                    f"着いたら自動で楕円開始（中心 ({center[0]:+.2f}, {center[1]:+.2f})）。e / c で中止")
                return
        self._start_ellipse()

    def _start_ellipse(self):
        sw = self.swarm
        self.pending_ellipse = False
        self.motion = (Ellipse(tuple(self.reference), self.heading, sw.ellipse_a, sw.ellipse_b,
                               sw.ellipse_period), self._now())
        c = self.motion[0].center
        self.get_logger().info(
            f"楕円軌道 開始: 中心=({c[0]:+.2f}, {c[1]:+.2f}) a={sw.ellipse_a} b={sw.ellipse_b} "
            f"周期={sw.ellipse_period}s | Leader の範囲 x∈[{c[0] - sw.ellipse_a:+.2f}, {c[0] + sw.ellipse_a:+.2f}] "
            f"y∈[{c[1] - sw.ellipse_b:+.2f}, {c[1] + sw.ellipse_b:+.2f}]（機体は隊形の半径ぶん外側まで動く）")

    def stop_motion(self):
        self.pending_ellipse = False
        self.motion = None
        self.velocity = [0.0, 0.0, 0.0]
        self.heading_rate = 0.0

    def go_to(self, goal, final_heading, label: str):
        sw = self.swarm
        self.stop_motion()
        g = GoTo(tuple(self.reference), self.heading, tuple(goal), sw.goto_speed, sw.turn_rate, final_heading)
        self.motion = (g, self._now())
        fh = f"{final_heading * 57.29578:+.0f}°" if final_heading is not None else "そのまま"
        self.get_logger().info(
            f"{label}: ゴール=({goal[0]:+.2f}, {goal[1]:+.2f}, {goal[2]:+.2f}) 到着後の向き={fh} "
            f"| 約 {g.duration:.1f} 秒（回転 {sw.turn_rate} rad/s・直進 {sw.goto_speed} m/s）")

    def _update_leader(self):
        now = time.monotonic()
        dt = min(now - self._last_tick, 0.2)
        self._last_tick = now
        if self.hold is not None:
            axis, sign, last = self.hold
            if now - last > HOLD_RELEASE or self.motion is not None:
                self.hold = None
                self.velocity = [0.0, 0.0, 0.0]
                self.publish_reference()          # 止まった位置をログに出す
            else:
                speed = self.swarm.hold_speed
                self.reference[axis] += sign * speed * dt
                self.velocity = [0.0, 0.0, 0.0]
                self.velocity[axis] = sign * speed   # 速度は配信するが、進行方向（隊形の向き）は変えない
                self.publish_reference(log=False)
        if self.motion is not None:
            traj, t0 = self.motion
            t = self._now() - t0
            st = traj.state(t, self.heading, self.swarm.heading_speed_threshold)
            self.reference = list(st.position)
            self.velocity = list(st.velocity)
            self.heading, self.heading_rate = st.heading, st.heading_rate
            self.publish_reference(log=False)
            if isinstance(traj, GoTo) and t >= traj.duration:
                if self.pending_ellipse:
                    self._start_ellipse()
                else:
                    self.stop_motion()
                    self.get_logger().info("ゴールに到着")

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = "formation_leader"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.reference
        qx, qy, qz, qw = quat_from_yaw(self.heading)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = self.velocity
        msg.twist.twist.angular.z = self.heading_rate
        self.leader_pub.publish(msg)

        # --- ドローンショー: 中心は Leader と同じ点、向きは隊形の回転角 φ（進行方向とは独立）---
        rate = self.swarm.show_spin_rate if self.spinning else 0.0
        self.spin_phase = math.atan2(math.sin(self.spin_phase + rate * dt), math.cos(self.spin_phase + rate * dt))
        show = Odometry()
        show.header = msg.header
        show.child_frame_id = "formation_show"
        show.pose.pose.position = msg.pose.pose.position
        qx, qy, qz, qw = quat_from_yaw(self.spin_phase)
        o = show.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        show.twist.twist.linear = msg.twist.twist.linear
        show.twist.twist.angular.z = rate
        self.show_pub.publish(show)

    def toggle_spin(self):
        self.spinning = not self.spinning
        if self.spinning:
            r = self.swarm.show_spin_rate
            period = 2 * math.pi / abs(r) if r else float("inf")
            self.get_logger().info(f"隊形の回転 開始: {r:+.3f} rad/s（1 回転 {period:.0f} 秒, 正 = 反時計回り）")
        else:
            self.get_logger().info(f"隊形の回転 停止（回転角 {math.degrees(self.spin_phase):+.0f}° で保持）")

    def publish_enable(self, value: bool):
        self.enabled = value
        self.enable_pub.publish(Bool(data=value))
        self.get_logger().info(f"formation enable = {value}")

    def move_reference(self, axis: int, sign: int):
        if self.motion is not None:
            self.get_logger().warn("楕円・ゴールへの移動中は手動移動できない（c で中止してから）")
            return
        self.reference[axis] += sign * self.swarm.reference_step
        self.publish_reference()

    def hold_move(self, axis: int, sign: int):
        """押しっぱなしのキーリピートが来るたびに呼ぶ。実際の移動は _update_leader（20Hz）で行う。"""
        if self.motion is not None:
            return
        if self.hold is None or self.hold[:2] != (axis, sign):
            self.get_logger().info(f"押しっぱなし移動: 軸{'xyz'[axis]} {'+' if sign > 0 else '-'} "
                                   f"{self.swarm.hold_speed} m/s（離すと止まる）")
        self.hold = (axis, sign, time.monotonic())

    def reset_reference(self):
        # 向きを一瞬で 0 に戻すと隊形が跳ぶので、ゴール指定と同じくゆっくり戻す
        self.go_to(self.swarm.reference_initial, 0.0, "初期位置へ戻る")

    def cancel_motion(self):
        if self.motion is not None:
            self.stop_motion()
            self.get_logger().info("移動を中止（その場で静止）")

    def _on_goal(self, msg: PoseStamped, topic: str, xy_only: bool):
        if msg.header.frame_id != self.world_frame:
            self.get_logger().error(f"{topic} の frame_id='{msg.header.frame_id}' は "
                                    f"'{self.world_frame}' ではないので無視（RViz の Fixed Frame を確認）")
            return
        p, q = msg.pose.position, msg.pose.orientation
        # RViz の 2D Goal Pose は z=0 で来るので高さは今のまま（xy と向きだけ使う）
        goal = (p.x, p.y, self.reference[2] if xy_only else p.z)
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            final_heading = 0.0   # 向き未指定（全成分 0）は +x とみなす
        else:
            final_heading = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.go_to(goal, final_heading, f"ゴール指定（{topic}）")

    # ------------------------------------------------------------------ 離着陸
    def send_all(self, cmd: str):
        for name, client in self.action_clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(f"[{name}] /{name}/tello_action が無い（driver / sim は起動してる？）")
                continue
            req = TelloAction.Request()
            req.cmd = cmd
            future = client.call_async(req)
            future.add_done_callback(lambda f, n=name: self._on_action_done(n, cmd, f))

    def _on_action_done(self, name: str, cmd: str, future):
        try:
            rc = future.result().rc
        except Exception as e:  # noqa: BLE001  サービス側の例外もログに残す
            self.get_logger().error(f"[{name}] '{cmd}' 失敗: {e}")
            return
        log = self.get_logger().info if rc == TelloAction.Response.OK else self.get_logger().warn
        log(f"[{name}] '{cmd}' → {RC_NAMES.get(rc, rc)}")


def get_key(timeout: float) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if rlist else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = FormationCommander()
    print(HELP)
    last_seen = {}   # キー → 最後にそのキーを受け取った時刻（無視したリピートも含めて更新する）
    try:
        while rclpy.ok():
            key = get_key(timeout=0.05)
            if key and key not in ("\x03", "X"):
                now = time.monotonic()
                repeated = now - last_seen.get(key, -1e9) < KEY_REPEAT_GAP
                last_seen[key] = now
                if repeated:
                    if key in MOVE_KEYS:
                        node.hold_move(*MOVE_KEYS[key])   # 押しっぱなし → 一定速度で移動
                    rclpy.spin_once(node, timeout_sec=0.0)
                    continue
            if key == "\x03":          # Ctrl-C
                break
            elif key == "t":
                node.send_all("takeoff")
            elif key == "l":
                node.send_all("land")
            elif key == "X":
                node.send_all("emergency")
            elif key == "g":
                node.publish_enable(True)
            elif key == "h":
                node.publish_enable(False)
            elif key == "z":
                node.reset_reference()
            elif key == "e":
                node.toggle_ellipse()
            elif key == "c":
                node.cancel_motion()
            elif key == "o":
                node.toggle_spin()
            elif key in MOVE_KEYS:
                node.move_reference(*MOVE_KEYS[key])
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_enable(False)
        for _ in range(10):            # enable=false を確実に届ける
            rclpy.spin_once(node, timeout_sec=0.02)
        # Ctrl-C は端末と launch の両方から届くので、後片付けに入ったら 2 回目以降の割り込みは無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
