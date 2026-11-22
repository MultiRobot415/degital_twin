#!/bin/bash
# 飛行データを rosbag で記録する（Sim / 実機 共通）。Ctrl-C で記録終了。
#
#   bash ~/Downloads/degital_twin-main/drone/tools/record_flight.sh [メモ]
#     → ~/flight_logs/<日時>_<メモ>/ に保存
#   あとでグラフにする:
#     ros2 run drone_control plot_flight ~/flight_logs/<日時>_<メモ>
#
# 記録するトピック（機体名は tello1, tello2, ... の形なら台数によらず自動で入る）:
#   /<name>/pose, /<name>/cmd_vel, /<name>/formation_target, /<name>/formation_velocity, /<name>/flight_data,
#   /formation/*（leader, reference, enable）
# ※ 他の端末と同じ ROS_DOMAIN_ID で実行すること（既定 42。違うと何も記録されない）。

source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "$0")/../ros2_ws" && pwd)/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

NAME=$(date +%Y%m%d_%H%M%S)${1:+_$1}
mkdir -p ~/flight_logs
OUT=~/flight_logs/$NAME
echo "記録先: $OUT （ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Ctrl-C で終了）"
exec ros2 bag record -o "$OUT" \
  -e '^/(tello[0-9]+/(pose|cmd_vel|formation_target|formation_velocity|flight_data)|formation/(leader|reference|enable))$'
