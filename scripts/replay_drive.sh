#!/usr/bin/env bash
# 기존 주행 모드(lidar_drive.launch.py)를 rosbag으로 재생하고 명령을 CSV로 기록한다.
# 리팩터링 전(baseline 태그)과 후에 같은 bag으로 한 번씩 실행하고 verify_refactor.py compare로 비교.
#
#   bash replay_drive.sh <bag 경로> <출력 csv> [재생 배속, 기본 0.5]
#   예) git checkout baseline-pre-refactor && colcon build ... && bash replay_drive.sh ~/bags/drive1 baseline_cmd.csv
#       git checkout exp/occlusion-speed   && colcon build ... && bash replay_drive.sh ~/bags/drive1 after_cmd.csv
#       python3 verify_refactor.py compare baseline_cmd.csv after_cmd.csv
#
# 토픽 이름은 check_vehicle.sh로 확정 전까지 환경변수로 바꿀 수 있다.
#   CLOUD_TOPIC (기본 /livox/lidar), ODOM_TOPIC (기본 /vesc/odom)
# bag에는 최소 CLOUD_TOPIC, ODOM_TOPIC이 있어야 한다. 기록된 명령 토픽은 재생하지 않는다.
# 실제 차량 드라이버(VESC)는 끈 상태에서 실행할 것: 재생 중 명령이 /commands/*로 나간다.

set -euo pipefail
BAG="${1:?bag 경로}"
OUT="${2:?출력 csv}"
RATE="${3:-0.5}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/livox/lidar}"
ODOM_TOPIC="${ODOM_TOPIC:-/vesc/odom}"
HERE="$(cd "$(dirname "$0")" && pwd)"

if ros2 node list 2>/dev/null | grep -qiE 'vesc_driver|vesc'; then
  echo "VESC 노드가 떠 있다. 재생 명령이 실제 모터로 갈 수 있으니 끄고 다시 실행." >&2
  exit 1
fi

ros2 launch my_algo lidar_drive.launch.py use_camera:=false use_sim_time:=true \
  use_metrics_logger:=false > "${OUT%.csv}_launch.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null || true' EXIT
sleep 5

# 파라미터도 함께 남겨 리팩터링 전후 값이 같은지 비교할 수 있게 한다.
for n in /lidar_reactive_drive /autonomous_drive /pointcloud_to_laserscan; do
  ros2 param dump "$n" > "${OUT%.csv}_params_$(basename $n).yaml" 2>&1 || true
done

python3 "$HERE/verify_refactor.py" record --output "$OUT" &
REC_PID=$!
sleep 2
ros2 bag play "$BAG" --clock --rate "$RATE" --topics "$CLOUD_TOPIC" "$ODOM_TOPIC"
wait $REC_PID
echo "완료: $OUT (파라미터 덤프: ${OUT%.csv}_params_*.yaml)"
