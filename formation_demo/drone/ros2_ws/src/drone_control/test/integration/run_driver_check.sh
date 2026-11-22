#!/bin/bash
# tello_swarm_driver を偽 Tello 3 台（127.0.0.2〜4）相手に検証する。機体・ネットワーク不要。
#   bash run_driver_check.sh
# 前提: colcon build 済み。UDP 8890 と 18889 が空いていること（実機ドライバを止めておく）。
set -m  # 非対話 bash だと背景プロセスが SIGINT を無視するため、ジョブ制御を有効にする
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-78}   # 研究室の他の ROS と混線しないように
LOGDIR=$(mktemp -d)

python3 "$HERE/fake_tello.py" --ip 127.0.0.2 --bat 81 --log "$LOGDIR/tello1.log" & F1=$!
python3 "$HERE/fake_tello.py" --ip 127.0.0.3 --bat 82 --log "$LOGDIR/tello2.log" & F2=$!
python3 "$HERE/fake_tello.py" --ip 127.0.0.4 --bat 83 --log "$LOGDIR/tello3.log" & F3=$!
ros2 run drone_control tello_swarm_driver --ros-args \
    -p swarm_file:="$HERE/swarm_loopback.yaml" -p local_command_port:=18889 > "$LOGDIR/driver.log" 2>&1 & D=$!
sleep 2

python3 "$HERE/driver_check.py" "$LOGDIR" "$F3"
RC=$?
kill -INT -$D; kill $F1 $F2 $F3; wait 2>/dev/null
echo "logs: $LOGDIR"
exit $RC
