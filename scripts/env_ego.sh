# 에고 Jetson 공통 환경. 모든 터미널에서:  source ~/f1tenth_ws/src/<my_algo 경로>/scripts/env_ego.sh
#   (설치본: source $(ros2 pkg prefix my_algo)/share/my_algo/scripts/env_ego.sh 는 ROS를 먼저 source해야 함)
# ROS2 Foxy (Ubuntu 20.04, Python 3.8). 순서: Foxy → vesc_ws(VESC 드라이버) → f1tenth_ws(Livox, my_algo) overlay

export EXP_ROS_DOMAIN_ID="${EXP_ROS_DOMAIN_ID:-37}"   # 두 Jetson·노트북 같은 값. 근처 다른 ROS 장비와 겹치면 바꾼다
export ROS_DOMAIN_ID="$EXP_ROS_DOMAIN_ID"
export WS="${WS:-$HOME/f1tenth_ws}"
export VESC_WS="${VESC_WS:-$HOME/vesc_ws}"

source /opt/ros/foxy/setup.bash
[ -f "$VESC_WS/install/setup.bash" ] && source "$VESC_WS/install/setup.bash" || echo "경고: $VESC_WS/install 없음"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" || echo "경고: $WS/install 없음 (먼저 colcon build)"

# my_algo 소스 레포 (meta.json git hash). --symlink-install 빌드면 설치본이 소스를 가리킨다.
if [ -z "${MY_ALGO_REPO:-}" ]; then
  _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd -P)"
  export MY_ALGO_REPO="$(cd "$_here/.." 2>/dev/null && git rev-parse --show-toplevel 2>/dev/null || echo "")"
fi
if ros2 pkg prefix my_algo >/dev/null 2>&1; then
  export S="$(ros2 pkg prefix my_algo)/share/my_algo/scripts"   # 스크립트 경로 단축
fi
echo "[env_ego] ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID MY_ALGO_REPO=${MY_ALGO_REPO:-?} S=${S:-?}"
