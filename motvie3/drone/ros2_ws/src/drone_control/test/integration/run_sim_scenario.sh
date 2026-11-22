#!/bin/bash
# Sim の結合テスト: swarm_sim + formation_agent×3 を起動し、離陸→開始→基準移動→停止→着陸 を自動で検証する。
#   bash run_sim_scenario.sh <formation> <収束待ち秒> <emulate_mocap true|false>
#   例: bash run_sim_scenario.sh leader_follower 20 true
#   収束待ちの目安: reference_tracking 15 / leader_follower 20 / consensus 60（遅いモードの時定数 ~13s）
set -m  # 非対話 bash だと背景プロセスが SIGINT を無視し、launch の子ノードが止まらないため
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-77}   # 研究室の他の ROS と混線しないように
LOG=$(mktemp)

ros2 launch drone_control swarm_sim.launch.py use_rviz:=false emulate_mocap:="$3" > "$LOG" 2>&1 & P1=$!
ros2 launch drone_control formation.launch.py formation:="$1" >> "$LOG" 2>&1 & P2=$!
sleep 4
timeout 300 python3 "$HERE/sim_scenario_check.py" "$1" "$2"
RC=$?
kill -INT -$P1 -$P2; wait $P1 $P2 2>/dev/null; sleep 1
LEFT=$(ps -eo args | grep -c "[l]ib/drone_control/")
[ "$LEFT" != "0" ] && echo "WARN: ノードが $LEFT 個残っている"
echo "log: $LOG"
exit $RC
