#!/bin/bash
# 論文 第3章の合意フォーメーションの Sim 結合テスト（静止 Leader → 楕円軌道 → 停止 → 着陸）。
#   bash run_ch3_scenario.sh <ch3_net1|ch3_net2> <emulate_mocap true|false>
set -m  # 非対話 bash だと背景プロセスが SIGINT を無視し、launch の子ノードが止まらないため
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-76}
LOG=$(mktemp)

ros2 launch drone_control swarm_sim.launch.py use_rviz:=false emulate_mocap:="$2" > "$LOG" 2>&1 & P1=$!
ros2 launch drone_control formation.launch.py formation:="$1" >> "$LOG" 2>&1 & P2=$!
sleep 4
timeout 300 python3 "$HERE/ch3_scenario_check.py" "$1"
RC=$?
kill -INT -$P1 -$P2; wait $P1 $P2 2>/dev/null; sleep 1
LEFT=$(ps -eo args | grep -c "[l]ib/drone_control/")
[ "$LEFT" != "0" ] && echo "WARN: ノードが $LEFT 個残っている"
echo "log: $LOG"
exit $RC
