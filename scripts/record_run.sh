#!/usr/bin/env bash
# 회차 기록: meta.json + params.yaml + rosbag2(sqlite3, Foxy 기본). bringup을 먼저 띄운 뒤 실행한다.
#
#   bash record_run.sh <mode> <method> <scenario> <d_e|na> <run_id>
#   예) bash record_run.sh exp P S1 1.5 3
#       bash record_run.sh exp B2 S3 na 1
# Ctrl+C로 기록을 끝내면 abort.json(데드맨 정지) 등을 meta.json에 합친다.
#
# 확인 전 토픽 이름은 환경변수로 바꾼다 (기본값은 미확인 추정값):
#   CLOUD_TOPIC(/livox/lidar) ODOM_TOPIC(/vesc/odom) RAW_IMU_TOPIC(/livox/imu)
#   VESC_STATE_TOPIC(/sensors/core) CAMERA_TOPIC(/camera/camera/color/image_raw/compressed)
#   EXTRA_TOPICS="..." 추가 기록
set -euo pipefail
[ $# -eq 5 ] || { sed -n '2,12p' "$0"; exit 1; }
MODE=$1 METHOD=$2 SCENARIO=$3 DE=$4 RUN_ID=$5
HERE="$(cd "$(dirname "$0")" && pwd)"

export CLOUD_TOPIC="${CLOUD_TOPIC:-/livox/lidar}"
export ODOM_TOPIC="${ODOM_TOPIC:-/vesc/odom}"
RAW_IMU_TOPIC="${RAW_IMU_TOPIC:-/livox/imu}"
export VESC_STATE_TOPIC="${VESC_STATE_TOPIC:-/sensors/core}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/camera/color/image_raw/compressed}"

# 발행자 점검: 예상 외 발행자(#2 등)가 있으면 회차 시작 거부 (exp 모드)
if [ "$MODE" = exp ]; then
  python3 "$HERE/check_preflight.py" || exit 1
fi

export TEST_V_REF="${TEST_V_REF:-}"   # bringup test_v_ref:= 를 썼다면 같은 값을 넣는다 (meta.json 기록)
RUN_DIR=$(python3 "$HERE/run_meta.py" start "$MODE" "$METHOD" "$SCENARIO" "$DE" "$RUN_ID" | tail -n 1)
echo "회차 폴더: $RUN_DIR"

# 실제 로드된 파라미터 전체 덤프 (실행 중인 my_algo/scan 노드)
: > "$RUN_DIR/params.yaml"
for n in $(ros2 node list 2>/dev/null | grep -E 'occupancy_grid_node|phantom_node|speed_layer_node|exp_aeb_node|exp_steering_node|exp_drive_mux|trigger_node|deskew_node|imu_republisher|pointcloud_to_laserscan|lidar_reactive_drive|autonomous_drive'); do
  echo "# ---- $n" >> "$RUN_DIR/params.yaml"
  ros2 param dump "$n" --print >> "$RUN_DIR/params.yaml" 2>&1 || echo "# dump 실패: $n" >> "$RUN_DIR/params.yaml"   # Foxy: --print
done

STORAGE=(-s sqlite3)   # Foxy 기본 저장소. 분석은 rosbags 0.9.23 (analysis/requirements.txt)

TOPICS=(
  "$CLOUD_TOPIC" "$ODOM_TOPIC" "$RAW_IMU_TOPIC" /imu /imu/linear_accel_base /imu/status
  "$VESC_STATE_TOPIC" /commands/motor/speed /commands/servo/position
  /exp/final_cmd /exp/v_cap /exp/v_raw /exp/v_cmd /exp/min_speed_flag /exp/stop_reason /exp/abort
  /exp/phantom_d /exp/rho /exp/phantoms /exp/phantom_debug
  /exp/grid /exp/boundary /exp/boundary_frame /exp/grid_debug
  /exp/steering_command /aeb/active /aeb/status /scan
  /trigger /trigger/info /popout/status /autonomous_mode /joy_active /joy
  /tf /tf_static "$CAMERA_TOPIC"
)
[ "${EXTRA_TOPICS:-}" ] && TOPICS+=($EXTRA_TOPICS)
[ "$MODE" = drive ] && TOPICS+=(/reactive/lidar_command)

finish() { trap - INT TERM; kill -INT "$REC" 2>/dev/null || true; wait "$REC" 2>/dev/null || true
           python3 "$HERE/run_meta.py" finish "$RUN_DIR"; }
trap finish INT TERM
ros2 bag record "${STORAGE[@]}" -o "$RUN_DIR/bag" "${TOPICS[@]}" &
REC=$!
echo "기록 중 (Ctrl+C로 종료)"
wait "$REC" || true
finish
