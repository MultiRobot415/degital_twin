# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""複数エージェント SHIELD launch：3機を1コマンドで立て、相互回避（自律交差）を検証する。

設計の肝（なぜこの構成か）:
    マルチエージェントは新アルゴリズム不要。シミュ run_multiagent.py と同じく
    「各機は“相手機の現在位置”を動く障害物として自分の SHIELD フィルタに渡す」だけ。
    つまり複数障害物の仕組みの上にそのまま乗る。実機側は配線(namespace)が全て:

      mocap driver ……… 1個だけ（共有）。/rigid_bodies に全剛体(機体1・機体2・静止障害物)が流れる
      RViz ……………… 1個だけ（共有）

      ＜機体ごと(×N)。すべて namespace /telloN 配下＞
        tello_driver       /telloN/cmd_vel 受け・実機Telloと通信（ローカルポートを機体毎にずらす）
        pose_bridge        /rigid_bodies(自IDを選ぶ) → /telloN/pose（body_frameも分ける）
        obstacle_bridge    /rigid_bodies(相手IDを選ぶ) → /telloN/shield/obstacles_state
        shield_filter      /telloN/cmd_vel_nom を安全な /telloN/cmd_vel に整形（相手を障害物扱い）
        position_controller  目標へ自走し /telloN/cmd_vel_nom を出す（★キーボード不要・自律）

    → キーボードで2機を手動操縦はしない。各機は position_controller で目標へ自走する
      （sim の goal_velocity(x, GOAL) に対応）。teleop は単機の手動確認用に別途使う。

ポート競合の注意（1PCで2機動かす肝）:
    tello_driver はローカルの command/data/video ポートを bind する。2台ぶん同じ既定値だと
    衝突して2台目が死ぬ。port_offset で機体ごとにずらす（既定: 1機目=0, 2機目=+10）。
    drone_port(8889) は Tello 側の宛先なので IP が違えば同じでよい。

起動（torch を使うので install を source した上で ros2 launch すること）:
    source /opt/ros/humble/setup.bash
    cd ~/research/drone/ros2_ws && source install/setup.bash
    ros2 launch drone_control multiagent.launch.py mode:=shield_multi

    安全のため既定は hold_on_start:=true（離陸しても各機その場ホールド）。
    各機を離陸させてから交差を始めるには、固定ゴールへ追従させる:
      ros2 launch drone_control multiagent.launch.py hold_on_start:=false
    あるいは個別に目標を送る（RViz の 2D Goal Pose を namespace 別に remap して使うか）:
      ros2 topic pub --once /tello1/goal_pose geometry_msgs/PoseStamped '{...}'

引数:
    mode:=shield_multi        shield_multi=全障害物同時(推奨) / shield=最近傍1個 / nominal=素CBF
    hold_on_start:=true       true=離陸後その場ホールド(安全) / false=goal_* へ即追従(交差開始)
    convert_y_up:=true        bringup と必ず一致（研究室設定は true）
    drone1_ip/drone2_ip/drone3_ip  各機の station モード IP（既定 .50/.51/.21。3機目は.21）
    drone1_id/drone2_id/drone3_id  各機の Motive Streaming ID（自機以外が自動で障害物になる）
    obstacle_radius:=0.10     相手機を障害物とみなす半径。robot_radius と足して中心間の最小距離になる
    robot_radius:=0.10        ドローン自体の半径。全障害物に効く。合算 R = 0.10+0.10 = 0.20m（既定）
    static_obstacle_radius:=0.10  会場に置いた静止障害物の半径（細め既定）。ID は static_obstacle_ids（既定[4,5]）
                              全機がこの静止障害物も避ける。ID を変える/増やすは launch 冒頭のリストを編集
    goal1_x/goal1_y           機体1(ID1)のゴール G1（既定 ( 0.7,-0.7) ＝ raw(z= 0.7,x= 0.7)）
    goal2_x/goal2_y           機体2(ID2)のゴール G2（既定 ( 0.7, 0.7) ＝ raw(z=-0.7,x= 0.7)）
    goal3_x/goal3_y           機体3(ID3)のゴール G3（既定 (-0.7,-0.7) ＝ raw(z= 0.7,x=-0.7)）
    flight_height:=0.9        3機共通の飛行高度[m]（既定 0.9。下限目安 0.6）
    wall_enable:=true         会場境界の壁を障害物として入れる（領域外に出さない）
    wall_x_min/x_max/y_min/y_max  壁の位置[m]（既定 ±1.2）。機体中心は各辺から
                              robot_radius 内側まで＝既定なら ±1.10 が限界
    wall_arc_radius:=200.0    壁を近似する巨大円の半径[m]（大きいほど直線に近い）
    beta:=3.0                 LSE smooth-min の鋭さ。★壁を入れるなら 3 以上が必須
                              （下げると過剰保守で u_max を超え、clip で制約違反する）
    slack_weight:=10.0        CBF制約のスラック重み。小さいほど緩む（<0 で無効＝硬い制約）
    slack_max:=0.05           緩めていい上限[m/s]。安全側の歯止め（<0 で無制限）
    u_max:=0.1                フィルタ出力上限[m/s]（実機の安定性優先。上げると姿勢が荒れる）
    use_rviz:=true            共有 RViz を出す
    enable_video:=false       カメラ映像を受けるか。既定 false（交差デモは映像不要）。
                              false だと streamon を送らず h264 デコードエラーが出ない上、
                              3機ぶんの映像UDPが WiFi を食わないので指令の遅延も減る
    python_bin:=<path>        torch入り python（既定 ~/env_isaaclab/bin/python）

■ 会場と配置
    ★会場では生mocapの (z, x) で会話することが多い。変換は ros_x=raw_x, ros_y=-raw_z。
    3機を配置し、中央付近で交差させる。

        機体          スタートS              ゴールG          （括弧内は生mocap (z,x)）
          tello1(ID1)  (-0.7,  0.7)  ─────▶  ( 0.7, -0.7)   S raw(-0.7,-0.7) G raw( 0.7, 0.7)
          tello2(ID2)  (-0.7, -0.7)  ─────▶  ( 0.7,  0.7)   S raw( 0.7,-0.7) G raw(-0.7, 0.7)
          tello3(ID3)  ( 0.7,  0.0)  ─────▶  (-0.7, -0.7)   S raw( 0,  0.7)  G raw( 0.7,-0.7)

    tello1 と tello2 は中央(0,0)で真正面から交差し、tello3 が右から入って両方を横切る。
    ※ G3 は S2 と同一点。tello2 が離脱した後に tello3 が入る。

    ※ スタックは「置いた位置から離陸→ゴールへ自走」なので、上のスタート座標に
      物理的に機体を置く（mocap で /telloN/pose が合っているか RViz で確認してから飛ばす）。
    ※ 一斉離着陸・S/G 一斉指令は multiagent_teleop（T離陸/L着陸/Bスタート/Cゴール）を使う。
      teleop の home_*/cross_* は上の S/G と一致させてある（既定で3機ぶん設定済み）。
"""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            GroupAction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg


def _drone_stack(ns, drone_ip, self_id, other_ids, port_offset,
                 goal_x, goal_y, *, mode, hold_on_start, convert_y_up,
                 obstacle_radius, robot_radius, u_max, beta, use_refined,
                 python_bin, static_ids, static_radius, flight_height,
                 wall_enable, wall_bounds_sub, wall_arc_radius,
                 slack_weight, slack_max, enable_video):
    """1機ぶんのノード群（driver/pose_bridge/obstacle_bridge/shield_filter/controller）を返す。

    トピックは /{ns}/... に揃える:
      - tello_driver は相対名(cmd_vel 等)なので PushRosNamespace で /{ns}/ 配下に入る
      - pose_bridge/obstacle_bridge は output_topic を /{ns}/... に明示
      - shield_filter/position_controller は絶対名(/tello/pose 等)を使うので -r で明示 remap
    """
    pose_topic = f"/{ns}/pose"
    nom_topic = f"/{ns}/cmd_vel_nom"
    cmd_topic = f"/{ns}/cmd_vel"
    obs_topic = f"/{ns}/shield/obstacles_state"
    goal_topic = f"/{ns}/goal_pose"

    # tello_driver：相対トピックなので namespace push で /{ns}/cmd_vel 等になる。
    # ローカル bind ポートを機体ごとにずらして衝突を防ぐ（drone_port は宛先なので共通でOK）。
    driver = GroupAction([
        PushRosNamespace(ns),
        Node(
            package="tello_driver",
            executable="tello_driver_main",
            name="tello_driver",
            output="screen",
            parameters=[{
                "drone_ip": ParameterValue(drone_ip, value_type=str),
                "command_port": 38065 + port_offset,
                "data_port": 8890 + port_offset,
                "video_port": 11111 + port_offset,
                # 交差デモは映像を使わない。false にすると streamon を送らず
                # VideoSocket も作らないので、h264 のデコードエラー
                # （"error decoding frame" / "concealing N DC..."）が消え、
                # 3機ぶんの映像UDPが WiFi を食わなくなる＝指令の遅延も減る。
                "enable_video": ParameterValue(enable_video, value_type=bool),
            }],
        ),
    ])

    # mocap → /{ns}/pose（自機IDを選ぶ）。body_frame を分けて TF 衝突を避ける。
    pose_bridge = Node(
        package="drone_control",
        executable="pose_bridge",
        name="pose_bridge",
        namespace=ns,
        output="screen",
        parameters=[{
            "rigid_body_id": ParameterValue(self_id, value_type=str),
            "convert_y_up": ParameterValue(convert_y_up, value_type=bool),
            "output_topic": pose_topic,
            "body_frame": ns,
        }],
    )

    # mocap → /{ns}/shield/obstacles_state（★相手機“全機”＋静止障害物のIDを選ぶ）。
    # obstacle_ids/obstacle_radii はリスト引数。相手機(N-1機, 半径=obstacle_radius)に加えて、
    # 会場に置いた静止障害物 static_ids(半径=static_radius) も同じリストに並べる。
    # substitution 連結で [id1,id2,...] を組む（LaunchConfiguration は実行時解決なので str()
    # 不可。リストで渡すと launch が連結。カンマも要素として挟む）。
    #   entries = 相手機（動的・obstacle_radius） + 静止障害物（static_radius）
    entries = [(oid, obstacle_radius) for oid in other_ids] \
        + [(sid, static_radius) for sid in static_ids]
    obs_ids_sub = ["obstacle_ids:=["]
    obs_radii_sub = ["obstacle_radii:=["]
    for i, (oid, rad) in enumerate(entries):
        if i:
            obs_ids_sub.append(",")
            obs_radii_sub.append(",")
        obs_ids_sub.append(oid)       # 相手機=LaunchConfiguration / 静止=文字列 どちらも可
        obs_radii_sub.append(rad)
    obs_ids_sub.append("]")
    obs_radii_sub.append("]")
    obstacle_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "drone_control", "obstacle_bridge", "--ros-args",
            "-r", f"__ns:=/{ns}",
            "-p", obs_ids_sub,
            "-p", obs_radii_sub,
            "-p", ["convert_y_up:=", convert_y_up],
            "-p", f"output_topic:={obs_topic}",
        ],
        output="screen",
    )

    # 安全フィルタ：torch が要るので venv python で -m 起動。絶対名を /{ns}/... へ remap。
    shield_filter = ExecuteProcess(
        cmd=[
            python_bin, "-m", "dist_learn.shield_filter_node", "--ros-args",
            "-r", f"__node:={ns}_shield_filter",
            "-r", f"/tello/pose:={pose_topic}",
            "-r", f"/cmd_vel_nom:={nom_topic}",
            "-r", f"/cmd_vel:={cmd_topic}",
            "-r", f"/shield/obstacles:=/{ns}/shield/obstacles",
            "-r", f"/shield/trajectory:=/{ns}/shield/trajectory",
            "-p", ["mode:=", mode],
            "-p", f"obstacle_topic:={obs_topic}",
            "-p", ["robot_radius:=", robot_radius],
            "-p", ["u_max:=", u_max],
            "-p", ["beta:=", beta],
            "-p", ["use_refined:=", use_refined],
            # 会場境界の壁（領域外＝mocap範囲外に出さない）。巨大円で半平面を近似し、
            # 既存の h_hat_multi / LSE smooth-min にそのまま相乗りする。
            "-p", ["wall_enable:=", wall_enable],
            "-p", wall_bounds_sub,
            "-p", ["wall_arc_radius:=", wall_arc_radius],
            # QP スラック：制約 a^T u >= b を b - s に緩め、コストに w*s^2 を足す。
            # デッドロック（u の予算が全部退避方向に食われる）でも前進成分を残す。
            "-p", ["slack_weight:=", slack_weight],
            "-p", ["slack_max:=", slack_max],
        ],
        output="screen",
    )

    # 自律ゴール追従：/{ns}/cmd_vel_nom を出す（= フィルタの“生指令”入力）。
    position_controller = Node(
        package="drone_control",
        executable="position_controller",
        name="position_controller",
        namespace=ns,
        output="screen",
        parameters=[{
            "goal_x": ParameterValue(goal_x, value_type=float),
            "goal_y": ParameterValue(goal_y, value_type=float),
            # 飛行高度[m]。hold_on_start=true でも「XYは現在地・高さはここ」へ上がる。
            # 離陸ゲート takeoff_height(0.3) より十分上に置くこと（下回ると制御が止まる）。
            "goal_z": ParameterValue(flight_height, value_type=float),
            "hold_on_start": ParameterValue(hold_on_start, value_type=bool),
        }],
        remappings=[
            ("/tello/pose", pose_topic),
            ("/goal_pose", goal_topic),
            ("/cmd_vel", nom_topic),   # ★生指令 → shield_filter へ
        ],
    )

    return [driver, pose_bridge, obstacle_bridge, shield_filter, position_controller]


def generate_launch_description():
    drone_share = get_package_share_directory("drone_control")
    rviz_config = os.path.join(drone_share, "rviz", "tello.rviz")
    default_mocap_params = os.path.join(
        get_package_share_directory("mocap4r2_optitrack_driver"),
        "config",
        "mocap4r2_optitrack_driver_params.yaml",
    )
    default_python = os.path.expanduser("~/env_isaaclab/bin/python")

    use_rviz = LaunchConfiguration("use_rviz")
    config_file = LaunchConfiguration("config_file")
    mode = LaunchConfiguration("mode")
    hold_on_start = LaunchConfiguration("hold_on_start")
    flight_height = LaunchConfiguration("flight_height")
    wall_enable = LaunchConfiguration("wall_enable")
    wall_arc_radius = LaunchConfiguration("wall_arc_radius")
    slack_weight = LaunchConfiguration("slack_weight")
    slack_max = LaunchConfiguration("slack_max")
    enable_video = LaunchConfiguration("enable_video")
    # wall_bounds は配列パラメータなので substitution を連結して [a,b,c,d] を組む
    # （obstacle_ids と同じ手口。LaunchConfiguration は実行時解決なので str() 不可）。
    wall_bounds_sub = [
        "wall_bounds:=[",
        LaunchConfiguration("wall_x_min"), ",", LaunchConfiguration("wall_x_max"), ",",
        LaunchConfiguration("wall_y_min"), ",", LaunchConfiguration("wall_y_max"), "]",
    ]
    convert_y_up = LaunchConfiguration("convert_y_up")
    obstacle_radius = LaunchConfiguration("obstacle_radius")
    robot_radius = LaunchConfiguration("robot_radius")
    u_max = LaunchConfiguration("u_max")
    beta = LaunchConfiguration("beta")
    use_refined = LaunchConfiguration("use_refined")
    python_bin = LaunchConfiguration("python_bin")
    drone1_ip = LaunchConfiguration("drone1_ip")
    drone2_ip = LaunchConfiguration("drone2_ip")
    drone3_ip = LaunchConfiguration("drone3_ip")
    drone1_id = LaunchConfiguration("drone1_id")
    drone2_id = LaunchConfiguration("drone2_id")
    drone3_id = LaunchConfiguration("drone3_id")
    static_obstacle_radius = LaunchConfiguration("static_obstacle_radius")

    # 会場に置いた静止障害物の Motive Streaming ID（全機がこれを障害物として避ける）。
    # 複数置くなら ["4", "5"] のように足す。半径は static_obstacle_radius 引数で変える。
    static_obstacle_ids = ["4", "5"]

    # --- 共有 mocap ドライバ（ライフサイクル）→ /rigid_bodies。bringup と同じ自動 configure→activate ---
    mocap_driver = LifecycleNode(
        name="mocap4r2_optitrack_driver_node",
        namespace="",
        package="mocap4r2_optitrack_driver",
        executable="mocap4r2_optitrack_driver_main",
        output="screen",
        parameters=[config_file],
    )
    mocap_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    mocap_activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mocap_driver,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matchers.matches_action(mocap_driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    common = dict(
        mode=mode, hold_on_start=hold_on_start, convert_y_up=convert_y_up,
        obstacle_radius=obstacle_radius, robot_radius=robot_radius, u_max=u_max,
        beta=beta, use_refined=use_refined, python_bin=python_bin,
        static_ids=static_obstacle_ids, static_radius=static_obstacle_radius,
        flight_height=flight_height,
        wall_enable=wall_enable, wall_bounds_sub=wall_bounds_sub,
        wall_arc_radius=wall_arc_radius,
        slack_weight=slack_weight, slack_max=slack_max,
        enable_video=enable_video,
    )

    # 機体1：自ID=drone1_id、相手=残り全機(drone2_id, drone3_id)を障害物に。交差ゴールは
    # 横ずれを少し付けて正面衝突の特異点＝デッドロックを避ける。
    stack1 = _drone_stack(
        ns="tello1", drone_ip=drone1_ip, self_id=drone1_id,
        other_ids=[drone2_id, drone3_id], port_offset=0,
        goal_x=LaunchConfiguration("goal1_x"), goal_y=LaunchConfiguration("goal1_y"),
        **common,
    )
    # 機体2：自ID=drone2_id、相手=残り全機(drone1_id, drone3_id)を障害物に。
    stack2 = _drone_stack(
        ns="tello2", drone_ip=drone2_ip, self_id=drone2_id,
        other_ids=[drone1_id, drone3_id], port_offset=10,
        goal_x=LaunchConfiguration("goal2_x"), goal_y=LaunchConfiguration("goal2_y"),
        **common,
    )
    # 機体3：自ID=drone3_id、相手=残り全機(drone1_id, drone2_id)を障害物に。
    stack3 = _drone_stack(
        ns="tello3", drone_ip=drone3_ip, self_id=drone3_id,
        other_ids=[drone1_id, drone2_id], port_offset=20,
        goal_x=LaunchConfiguration("goal3_x"), goal_y=LaunchConfiguration("goal3_y"),
        **common,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # カメラ映像。交差デモは使わないので既定 false。
        # true に戻すと h264 デコードエラーのログが再び出る（無害だが騒がしい）。
        DeclareLaunchArgument("enable_video", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_mocap_params),
        DeclareLaunchArgument("mode", default_value="shield_multi"),
        DeclareLaunchArgument("hold_on_start", default_value="true"),
        # 3機共通の飛行高度[m]。全機同一にして XY 平面で交差させる（SHIELD は2D）。
        # 下限の目安 0.6m：離陸ゲート takeoff_height=0.3 との余裕と、Tello の
        # 地面効果（自機ダウンウォッシュの床反射）で 0.5m 以下は姿勢が荒れる。
        # これより低くしたい場合は position_controller の takeoff_height も下げる。
        DeclareLaunchArgument("flight_height", default_value="0.9"),
        DeclareLaunchArgument("convert_y_up", default_value="true"),
        # 会場が約2m×2m(x,y∈[-1,1])と狭いので安全半径を詰める。robot_radius はドローン
        # 自体の半径で、全障害物（機体どうし・静止障害物）のキープアウトに効く。
        # 機体どうしの中心間 R = obstacle_radius + robot_radius = 0.10 + 0.10 = 0.20m。
        # 静止障害物も static_obstacle_radius + robot_radius = 0.10 + 0.10 = 0.20m。
        # （過拘束による外逃げ/デッドロックを避けるため robot_radius も 0.25→0.10 に縮小）
        # obstacle_radius: 相手機との中心間必要距離は
        # R = obstacle_radius + robot_radius = 0.10 + 0.10 = 0.20 m。
        # 会場が狭く安全半径が大きいと外逃げ/デッドロックするため 0.10 のまま。
        # 留意: Tello はプロペラ込みで対角 ~0.18m なので R=0.20 だと機体間の空きは
        # 20mm ほど。同高度で 0.2m まで寄ると互いのダウンウォッシュを受ける。
        # （高度を機体ごとにずらせば XY の回避はそのままで干渉だけ避けられる）
        DeclareLaunchArgument("obstacle_radius", default_value="0.10"),
        DeclareLaunchArgument("robot_radius", default_value="0.10"),
        # 静止障害物（会場に置いた物体, Motive ID は static_obstacle_ids で指定）の半径[m]。
        # 全機がこの半径で ID4 等を避ける。合算距離 = static_obstacle_radius + robot_radius。
        # 既定0.10m（細め）＝ 中心間 0.10+0.25=0.35m。太くするなら引数で上書き。
        DeclareLaunchArgument("static_obstacle_radius", default_value="0.10"),
        # u_max: フィルタ出力の上限[m/s]。実機で 0.4 は姿勢が安定しなかったため 0.1。
        # 注意（シミュとの食い違い）: real_xy.npz から同定した外乱(1σ=0.175 m/s,
        # 相関時間0.6-1.1s)を入れると u_max=0.1 は成立しない（合格2/10）。ただしその
        # データは mean|u|=0.126・max0.5 の強い指令で収集され外れ値も残る(max|d|=0.725)
        # ので穏やかな飛行に対しては悲観側。かつ u_max を上げると Tello の傾きが増えて
        # 外乱自体が増える（同定した開ループモデルはこの効果を含まない）。
        # 実機の観測を優先して 0.1。ドリフトが 1σ 0.07 m/s 以下に収まっていれば成立する
        # （シミュでの成立条件。log/traj の npz で確認できる）。
        DeclareLaunchArgument("u_max", default_value="0.1"),
        # 交差ゴール。中央(0,0)付近で3機が交差するように置く。
        # ★座標は ROS(x,y)。会場で使う生mocapは (z,x) 表記なので必ず変換すること:
        #     ros_x = raw_x ,  ros_y = -raw_z
        # 各機のゴール G（生mocap(z,x) → ROS(x,y)）:
        #   ID1: G1 raw(z=+0.7, x=+0.7) → ( 0.7, -0.7)
        #   ID2: G2 raw(z=-0.7, x=+0.7) → ( 0.7,  0.7)
        #   ID3: G3 raw(z=+0.7, x=-0.7) → (-0.7, -0.7)
        # 壁 ±1.2・機体中心の限界 ±1.10 なので 0.7 は余裕で収まる。
        # ※ G3(ROS -0.7,-0.7) は S2(ROS -0.7,-0.7) と同一点。tello2 が出発した後に
        #   tello3 が入る前提。tello2 の離脱が遅いと tello3 がそこで待たされる。
        DeclareLaunchArgument("goal1_x", default_value="0.7"),
        DeclareLaunchArgument("goal1_y", default_value="-0.7"),
        DeclareLaunchArgument("goal2_x", default_value="0.7"),
        DeclareLaunchArgument("goal2_y", default_value="0.7"),
        DeclareLaunchArgument("goal3_x", default_value="-0.7"),
        DeclareLaunchArgument("goal3_y", default_value="-0.7"),
        # ★beta は LSE smooth-min の鋭さ。h_hat_multi は障害物 N 個の h が同程度のとき
        # 真の min より log(N)/beta だけ低く出る（＝過剰に危険と判断する）。この余分な
        # 保守性は b に (1-alpha)*log(N)/beta/dt として乗り、必要な |u| を押し上げる。
        # 実測（robot_r=0.10, alpha=0.9, dt=0.05, 交差シーン）の「制約を満たすのに
        # 必要な |u|」。u_max=0.1 を超えると clip で黙って制約違反になる:
        #     beta=1.0 壁なし(N=2) → 4.0 m/s   壁あり(N=6) → 32.1 m/s   ← 両方 NG
        #     beta=2.0 壁なし      → 充足        壁あり      →  5.0 m/s   ← 壁が NG
        #     beta=3.0 壁なし      → 充足        壁あり      → 充足        ← OK
        # よって壁を入れるなら beta>=3 が必須。beta を上げる代償は
        # lambda_max_multi=lam*gamma^2*(1+beta*lam) 経由の分散補正だが dt^2 が掛かるので
        # 実質無視できる（beta=3 で 4e-5 程度）。従来挙動に戻すなら beta:=1.0。
        DeclareLaunchArgument("beta", default_value="3.0"),
        # --- 会場境界の壁 ---
        DeclareLaunchArgument("wall_enable", default_value="true"),
        # 壁の位置[m]（ROS(x,y)）。機体“中心”は各辺から robot_radius 内側までに制限される
        # ので、既定 ±1.2・robot_radius=0.10 なら中心は ±1.10 までしか行けない。
        # ※ ±1.0 だとシミュレーションで壁余裕が -0.007 m（わずかにはみ出す）だったため
        #   ±1.2 に広げてある。mocap の観測範囲内に収まっているかは会場で要確認。
        DeclareLaunchArgument("wall_x_min", default_value="-1.2"),
        DeclareLaunchArgument("wall_x_max", default_value="1.2"),
        DeclareLaunchArgument("wall_y_min", default_value="-1.2"),
        DeclareLaunchArgument("wall_y_max", default_value="1.2"),
        # 壁を近似する巨大円の半径[m]。会場±1mでの誤差は R=50m→10mm, R=200m→2.5mm。
        DeclareLaunchArgument("wall_arc_radius", default_value="200.0"),
        # --- QP スラック（デッドロックでも入力を出す）---
        # 小さいほど緩みやすい。目安 100以上≒ほぼ硬い / 10=控えめ / 1=かなり緩い。
        # <0 で無効＝従来どおりの硬い制約。
        DeclareLaunchArgument("slack_weight", default_value="10.0"),
        # 緩めていい上限[m/s]。h は1ステップ最大 dt*slack_max だけ余分に減る
        # （0.05 m/s × dt 0.05s = 2.5mm/step）。<0 で無制限。
        DeclareLaunchArgument("slack_max", default_value="0.05"),
        DeclareLaunchArgument("use_refined", default_value="true"),
        DeclareLaunchArgument("python_bin", default_value=default_python),
        # 実機IP↔SN（2026-08-30 疎通確認）:
        #   .50 = SN 0TQZK8PED02YFL   .51 = SN 0TQ3GBG00SS131   .21 = SN 0TQDG3KEDBJP36
        # ※ 3機目は DHCP で .21 を取得（.52 ではない）。IPが変わったら要更新。
        DeclareLaunchArgument("drone1_ip", default_value="192.168.11.50"),
        DeclareLaunchArgument("drone2_ip", default_value="192.168.11.51"),
        DeclareLaunchArgument("drone3_ip", default_value="192.168.11.21"),
        DeclareLaunchArgument("drone1_id", default_value="1"),
        DeclareLaunchArgument("drone2_id", default_value="2"),
        DeclareLaunchArgument("drone3_id", default_value="3"),

        mocap_driver,
        mocap_configure,
        mocap_activate_on_inactive,
        rviz,

        *stack1,
        *stack2,
        *stack3,
    ])
