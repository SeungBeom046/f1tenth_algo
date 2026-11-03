#!/usr/bin/env bash
# 오프라인 재생 비교: 한 회차 bag으로 여러 방법의 명령 속도를 다시 계산한다.
#
#   bash offline_replay.sh <run_dir> [방법 ...]     # 기본 B1 B2 P
#   python3 analysis/offline_compare.py <run_dir>   # 위치별 명령 속도·손실 비교
#
# 한계 (개루프): 에고 궤적은 녹화된 것 그대로다. 다른 방법이 냈을 속도로 실제 달렸다면 점군·위치가
# 달라졌을 것이므로, 이 결과는 "같은 관측에서 배치 규칙만 바꿨을 때의 명령 차이"로만 해석한다.
# 실행 중인 VESC/센서 드라이버는 끈다 (bag의 토픽과 섞이지 않게). 명령 발행 노드는 띄우지 않는다.
set -euo pipefail
RUN_DIR="${1:?run_dir}"; shift
METHODS=(B1 B2 P)
[ $# -gt 0 ] && METHODS=("$@")
CLOUD_TOPIC="${CLOUD_TOPIC:-$(python3 -c "import json;print(json.load(open('$RUN_DIR/meta.json')).get('cloud_topic','/livox/lidar'))")}"
ODOM_TOPIC="$(python3 -c "import json;print(json.load(open('$RUN_DIR/meta.json')).get('odom_topic','/vesc/odom'))")"
SCENARIO="$(python3 -c "import json;print(json.load(open('$RUN_DIR/meta.json'))['scenario'])")"
QOS="$(ros2 pkg prefix my_algo)/share/my_algo/config/replay_qos.yaml"
mkdir -p "$RUN_DIR/replay"
for M in "${METHODS[@]}"; do
  OUT="$RUN_DIR/replay/$M"
  rm -rf "$OUT"
  ros2 launch my_algo exp_perception.launch.py method:=$M scenario:=$SCENARIO run_dir:="$RUN_DIR" \
    cloud_topic:="$CLOUD_TOPIC" > "$RUN_DIR/replay/${M}_launch.log" 2>&1 &
  L=$!
  sleep 4
  ros2 bag record -o "$OUT" /exp/v_raw /exp/v_cap /exp/v_cmd /exp/phantom_d /exp/rho /exp/phantoms \
    --use-sim-time > /dev/null 2>&1 &
  R=$!
  sleep 2
  ros2 bag play "$RUN_DIR/bag" --clock --qos-profile-overrides-path "$QOS" \
    --topics "$CLOUD_TOPIC" "$ODOM_TOPIC" /tf /tf_static /autonomous_mode
  sleep 1
  kill -INT $R $L 2>/dev/null || true
  wait $R $L 2>/dev/null || true
  echo "$M 재생 완료: $OUT"
done
