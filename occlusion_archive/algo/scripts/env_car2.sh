# #2 Jetson 공통 환경:  source <my_algo 경로>/scripts/env_car2.sh
# #2에서는 car2.launch.py만 띄운다 (VESC 드라이버 /car2 + popout 노드). Livox·정적 TF·joy 금지.
export EXP_ROS_DOMAIN_ID="${EXP_ROS_DOMAIN_ID:-37}"   # env_ego.sh와 같은 값
export ROS_DOMAIN_ID="$EXP_ROS_DOMAIN_ID"
export WS="${WS:-$HOME/f1tenth_ws}"
export VESC_WS="${VESC_WS:-$HOME/vesc_ws}"
source /opt/ros/foxy/setup.bash
[ -f "$VESC_WS/install/setup.bash" ] && source "$VESC_WS/install/setup.bash" || echo "경고: $VESC_WS/install 없음"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" || echo "경고: $WS/install 없음 (먼저 colcon build)"
export CAR2_VESC_CONFIG="${CAR2_VESC_CONFIG:-$HOME/car2_vesc.yaml}"   # #2 VESC 설정 (포트 등) TBD(세션 1)
echo "[env_car2] ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID CAR2_VESC_CONFIG=$CAR2_VESC_CONFIG"
