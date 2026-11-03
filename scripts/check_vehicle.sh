#!/usr/bin/env bash
# 실차 토픽/드라이버/TF 점검. 결과 로그 파일 전체를 붙여 주면 토픽 이름을 확정한다.
#
# 사용 (Jetson, 평소 주행처럼 Livox/VESC/카메라 드라이버를 모두 띄운 상태, 차는 정지):
#   source /opt/ros/$ROS_DISTRO/setup.bash && source ~/f1tenth_ws/install/setup.bash
#   bash scripts/check_vehicle.sh            # 로그: ./check_vehicle_<host>_<시각>.log
#   HZ_SEC=8 bash scripts/check_vehicle.sh   # hz 측정 시간 변경
#
# 이 스크립트는 아무것도 발행하지 않는다 (읽기 전용).

set -u
HZ_SEC="${HZ_SEC:-6}"
LOG="check_vehicle_$(hostname)_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG") 2>&1

section() { printf '\n========== %s ==========\n' "$*"; }
run() { printf '$ %s\n' "$*"; timeout -s INT 15 "$@" 2>&1; printf '\n'; }

section "환경"
echo "host=$(hostname) date=$(date -Is)"
echo "ROS_DISTRO=${ROS_DISTRO:-unset} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset (=0)}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
uname -a
command -v chronyc >/dev/null && { run chronyc tracking; run chronyc sources -v; } \
  || echo "chronyc 없음 (chrony 미설치)"

section "패키지 존재 여부"
ros2 pkg list 2>/dev/null | grep -E \
  'livox|vesc|pointcloud_to_laserscan|robot_localization|tf2_tools|tf2_ros|rosbag2_storage_mcap|realsense|image_transport|compressed_image_transport|sensor_msgs_py' \
  || echo "(해당 패키지 없음)"

section "노드 목록"
run ros2 node list

section "토픽 목록 (타입 포함)"
TOPICS="$(timeout 15 ros2 topic list -t 2>/dev/null)"
echo "$TOPICS"

# 타입별 토픽 자동 탐지
topics_of() { echo "$TOPICS" | grep -E "\[$1\]" | awk '{print $1}'; }
PC_TOPICS="$(topics_of 'sensor_msgs/msg/PointCloud2')"
LIVOX_CUSTOM="$(topics_of 'livox_ros_driver2/msg/CustomMsg|livox_interfaces/msg/CustomMsg')"
IMU_TOPICS="$(topics_of 'sensor_msgs/msg/Imu')"
ODOM_TOPICS="$(topics_of 'nav_msgs/msg/Odometry')"
VESC_TOPICS="$(echo "$TOPICS" | grep -E '\[vesc_msgs/' | awk '{print $1}')"
SCAN_TOPICS="$(topics_of 'sensor_msgs/msg/LaserScan')"
IMG_TOPICS="$(topics_of 'sensor_msgs/msg/Image|sensor_msgs/msg/CompressedImage')"
CMD_TOPICS="$(echo "$TOPICS" | grep -E '^/commands/|ackermann' | awk '{print $1}')"

section "토픽 주기 (각 ${HZ_SEC}s)"
echo "참고: ros2 topic hz는 python이라 큰 PointCloud2에서 실제보다 낮게 나올 수 있음"
for t in $PC_TOPICS $LIVOX_CUSTOM $IMU_TOPICS $ODOM_TOPICS $VESC_TOPICS $SCAN_TOPICS $IMG_TOPICS $CMD_TOPICS /tf /tf_static; do
  echo "--- $t"
  timeout -s INT "$HZ_SEC" ros2 topic hz --window 100 "$t" 2>&1 | tail -n 3
done

section "IMU 원본 1회 echo"
for t in $IMU_TOPICS; do run ros2 topic echo --once "$t"; done

section "IMU 정지 통계 (단위 판정: |a| 평균 ~1.0 = g, ~9.8 = m/s^2)"
for t in $IMU_TOPICS; do
python3 - "$t" <<'PY'
import math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
topic = sys.argv[1]
rclpy.init(); node = rclpy.create_node('check_imu_stats')
acc, gyr, stamps, frames = [], [], [], set()
def cb(m):
    a, g = m.linear_acceleration, m.angular_velocity
    acc.append((a.x, a.y, a.z)); gyr.append((g.x, g.y, g.z))
    stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); frames.add(m.header.frame_id)
node.create_subscription(Imu, topic, cb, qos_profile_sensor_data)
end = time.time() + 3.0
while time.time() < end: rclpy.spin_once(node, timeout_sec=0.05)
n = len(acc)
print(f'{topic}: {n} samples / 3 s, frame_id={sorted(frames)}')
if n:
    mean = lambda v, i: sum(x[i] for x in v) / len(v)
    am = [mean(acc, i) for i in range(3)]; gm = [mean(gyr, i) for i in range(3)]
    print(f'  accel mean = ({am[0]:+.4f}, {am[1]:+.4f}, {am[2]:+.4f})  |a| = {math.sqrt(sum(x*x for x in am)):.4f}')
    print(f'  gyro  mean = ({gm[0]:+.5f}, {gm[1]:+.5f}, {gm[2]:+.5f})')
    if n > 1:
        dt = [b - a for a, b in zip(stamps, stamps[1:])]
        print(f'  header stamp rate = {(n - 1) / max(1e-9, stamps[-1] - stamps[0]):.1f} Hz, '
              f'dt min/max = {min(dt)*1e3:.2f}/{max(dt)*1e3:.2f} ms')
    print(f'  wall-clock vs header stamp offset = {time.time() - stamps[-1]:+.3f} s')
node.destroy_node(); rclpy.shutdown()
PY
done

section "점군 필드 (deskew용 점별 타임스탬프 확인)"
for t in $PC_TOPICS; do
  echo "--- $t"
  python3 - "$t" <<'PY'
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
rclpy.init(); node = rclpy.create_node('check_pc_fields'); got = []
node.create_subscription(PointCloud2, sys.argv[1], lambda m: got.append(m), qos_profile_sensor_data)
end = time.time() + 5.0
while not got and time.time() < end: rclpy.spin_once(node, timeout_sec=0.1)
if got:
    m = got[0]
    print(f'  frame_id={m.header.frame_id} height={m.height} width={m.width} point_step={m.point_step} is_dense={m.is_dense}')
    for f in m.fields: print(f'  field {f.name:12s} offset={f.offset:3d} datatype={f.datatype} count={f.count}')
    print(f'  header stamp vs wall-clock = {time.time() - (m.header.stamp.sec + m.header.stamp.nanosec*1e-9):+.3f} s')
else:
    print('  수신 실패')
node.destroy_node(); rclpy.shutdown()
PY
done
for t in $LIVOX_CUSTOM; do
  echo "--- $t (CustomMsg: 점별 offset_time 포함 여부)"
  run ros2 interface show "$(echo "$TOPICS" | grep -E "^$t " | sed -E 's/.*\[(.*)\]/\1/')"
done

section "TF 발행 노드"
run ros2 topic info -v /tf
run ros2 topic info -v /tf_static
run ros2 topic echo --once /tf_static
echo "--- odom -> base_link (3 s)"
timeout -s INT 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -n 12

section "드라이버 파라미터 (livox / vesc / odom / laserscan 노드)"
for n in $(timeout 10 ros2 node list 2>/dev/null | grep -Ei 'livox|vesc|odom|ackermann|laserscan'); do
  echo "--- $n"
  timeout 15 ros2 param dump "$n" 2>&1
done

section "속도 환산 계수 (명령 ERPM_GAIN vs odom speed_to_erpm_gain)"
echo "my_algo vehicle_utils.py:"
python3 -c "from my_algo.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM; print(f'  ERPM_GAIN={ERPM_GAIN} MIN_DRIVING_RPM={MIN_DRIVING_RPM} V_MIN={MIN_DRIVING_RPM/ERPM_GAIN:.4f} m/s')" 2>&1
echo "실행 중 노드 파라미터:"
for n in $(timeout 10 ros2 node list 2>/dev/null | grep -Ei 'vesc|odom|ackermann'); do
  for p in speed_to_erpm_gain speed_to_erpm_offset steering_angle_to_servo_gain steering_angle_to_servo_offset wheelbase publish_tf odom_frame base_frame use_servo_cmd_to_calc_angular_velocity; do
    v=$(timeout 5 ros2 param get "$n" "$p" 2>/dev/null) && echo "  $n $p: $v"
  done
done
echo "설정 파일 위치 (speed_to_erpm_gain 검색):"
grep -rn --include='*.yaml' --include='*.py' 'speed_to_erpm_gain' ~ /opt/ros 2>/dev/null | grep -v '/build/\|/log/' | head -n 20

section "VESC 상태 토픽 1회 echo (속도, 전압 필드 확인)"
for t in $VESC_TOPICS; do run ros2 topic echo --once "$t"; done

section "Odometry 1회 echo (frame_id, child_frame_id 확인)"
for t in $ODOM_TOPICS; do run ros2 topic echo --once "$t"; done

section "프로세스와 부하 (YOLO, python 노드)"
ps -eo pid,pcpu,pmem,etime,args --sort=-pcpu | grep -Ei 'ros|python|yolo|livox|vesc|realsense' | grep -v grep | head -n 30
command -v tegrastats >/dev/null && timeout 3 tegrastats --interval 1000 | head -n 2

section "디스크 (rosbag 기록 위치)"
df -h ~ /tmp 2>/dev/null
ros2 bag record --help 2>/dev/null | grep -A2 -- '--storage' | head -n 6

echo
echo "완료. 로그: $LOG"
