"""기본 주행 (main에서 온 실차 LiDAR 주행)과 두 연구가 함께 쓰는 유틸.

lidar_reactive_drive → autonomous_drive(AEB) → VESC 명령. 실행 파일 이름은 setup.py entry_points (ros2 run my_algo <이름>).
vehicle_utils(속도·서보 변환, 최소 속도 규칙), aeb(AEB 판정)는 추월 실험(my_algo.aur)과 시뮬도 쓴다.
"""
