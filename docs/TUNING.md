# 조정 파라미터 (TUNE 태그)

이 파일은 `scripts/gen_tuning_doc.py`가 만든다. 손으로 고치지 말고, 값·태그를 바꾼 뒤 커밋 전에 다시 실행한다.
라인 번호가 어긋나 보이면 직접 찾는다:

    grep -rn "TUNE:" config my_algo sim scripts

태그 형식: `# TUNE: <설명> | 단위 | 현재값 근거(임시/측정/기획안)`.
"기획안" 값은 가정값이며 측정값이 아니다. 권장 범위도 측정 전 추정이다.

| 파일 | 라인 | 파라미터 | 현재값 | 단위 | 의미 | 근거 | 바꿀 때 영향 | 권장 범위 |
|---|---|---|---|---|---|---|---|---|
| config/exp_common.yaml | 7 | `t_d` | 0.15 | s | 제동 지연 | 기획안 (보정 실험으로 대체) | 키우면 d_safe·d*가 커지고 v_lim이 작아진다. AEB(relative)와 같이 쓴다 | 0.10~0.30 |
| config/exp_common.yaml | 8 | `a_brake` | 3.0 | m/s^2 | 최대 감속도 | 기획안 (보정 실험으로 대체) | 줄이면 d_safe가 커지고 접근이 느려진다. 실제보다 크면 급정지 때 여유 m을 못 지킨다 | 2.0~4.0 |
| config/exp_common.yaml | 9 | `m_stop` | 0.15 | m | 정지 후 남길 여유 m | 기획안 | 정지 후 간격, MERGE 시작 조건(선행차 앞면이 뒤범퍼보다 m 뒤). 키우면 d*가 멀어진다 | 0.10~0.25 |
| config/exp_common.yaml | 10 | `eps_follow` | 0.05 | m | 추종 여유 ε | 기획안 | v_lim과 hold 판정의 여유. 키우면 d*가 멀어지고 hold가 줄어든다 | 0.03~0.10 |
| config/exp_common.yaml | 12 | `belief_model` | survival | - | belief 모델 original(기획안 식) / survival(기획안 변경) | 기획안 변경 | original은 Y를 NY로 오판(폐루프 115/200), survival은 188/200 | survival |
| config/exp_common.yaml | 13 | `d_y_min` | 0.6 | m | π(d) 분포 하한, 선행차 양보 반응 거리 최소 | 기획안 | survival에서 NY 확정 거리가 d_y_min + (1−η)(d_y_max − d_y_min)이 된다. 선행차 d_y 추첨 범위이기도 하다 | 0.4~0.8 |
| config/exp_common.yaml | 14 | `d_y_max` | 1.2 | m | π(d) 분포 상한, 선행차 양보 반응 거리 최대 | 기획안 | 이 거리 밖에서는 belief가 변하지 않는다. d* < d_y_max여야 탐색이 된다 | 1.0~1.5 |
| config/exp_common.yaml | 15 | `v_lat` | 0.5 | m/s | 양보 시 횡속도 (관측 모델, survival의 감지 지연 τ = y_yield_thr / v_lat) | 기획안 | original: 양보 우도의 평균 변위. survival: 감지 지연 τ. 실제보다 크면 τ를 짧게 봐서 NY 오판이 는다 | 0.3~0.6 |
| config/exp_common.yaml | 16 | `sigma_z` | 0.02 | m | 횡변위 관측 노이즈 σ (original 모델만 사용) | 기획안 (보정 실험으로 대체) | original만: 작을수록 scan 하나의 증거가 커진다 | 0.01~0.05 |
| config/exp_common.yaml | 17 | `eta` | 0.9 | - | 판단 임계 확률 η | 기획안 | 키우면 판단이 늦고 오판이 준다. survival의 Y 놓침 비율이 약 1−η | 0.8~0.95 |
| config/exp_common.yaml | 18 | `lambda_clip` | 20.0 | - | 로그 오즈 λ 클리핑 | 기획안 | λ 포화. 줄이면 반대 증거에 빨리 뒤집힌다 (original만 의미 있음) | 10~30 |
| config/exp_common.yaml | 19 | `y_yield_thr` | 0.08 | m | (survival) 양보로 보는 누적 횡변위, 4σ | 임시 | 줄이면 양보를 빨리 감지하지만 선행차 흔들림을 양보로 본다 (NY 추월 진입 위험) | 0.06~0.12 (3σ 이상) |
| config/exp_common.yaml | 20 | `p_false_yield` | 0.01 | - | (survival) NY인데 한 회차 안에 임계값을 넘을 확률 | 임시 | 키우면 감지 직후 b가 낮아 π(d)가 클 때만 Y로 확정된다 | 0.001~0.05 |
| config/exp_common.yaml | 21 | `T_max` | 8.0 | s | PROBE 최대 시간, 넘으면 FOLLOW | 기획안 | 줄이면 추월 포기가 빨라진다. 통로 길이 / v_B보다 짧아야 한다 | 5~12 |
| config/exp_common.yaml | 23 | `w_car` | 0.27 | m | 차폭 w | 측정 | δ, 추월 가능 판정, PASS 가속 조건 | 측정값 고정 |
| config/exp_common.yaml | 24 | `c_lat` | 0.10 | m | 횡 여유 c | 기획안 | 키우면 δ가 줄고(벽에서 멀어짐) 추월 가능 판정이 엄격해진다. 추월 차선 벽 거리 = w/2 + c | 0.05~0.15 |
| config/exp_common.yaml | 25 | `W` | null | m | 통로 폭 W, null이면 scan 추정값 사용 | 임시 (실험 장소에서 측정) | null이면 scan 추정. 고정하면 벽 제거와 δ가 흔들리지 않는다 | 0.84~1.3 (실측) |
| config/exp_common.yaml | 26 | `lane_offset` | null | m | 차선 횡 오프셋 δ, null이면 W/2 − w/2 − c | 기획안 | 직접 주면 W와 무관하게 차선 위치를 고정한다 | null |
| config/exp_common.yaml | 28 | `v_B_nominal` | 0.9 | m/s | 선행차 목표 속도 | 기획안 | d*와 접근 속도 차. v_min(0.867)보다 커야 한다 | 0.87~1.1 |
| config/exp_common.yaml | 29 | `v_pass` | 1.5 | m/s | 추월 속도 | 기획안 | 키우면 추월이 빨라지지만 접근 중 v_lim에 걸리는 시간이 늘고 제동 거리가 길어진다 | 1.3~1.8 |
| config/exp_common.yaml | 30 | `erpm_gain` | 4614.0 | ERPM/(m/s) | m/s → ERPM 환산 | 측정 (vehicle_utils.ERPM_GAIN, 테이프 구간으로 재검증) | 속도 환산 전체. 틀리면 v_E, v_B, h가 모두 틀어진다 | 측정값 고정 |
| config/exp_common.yaml | 31 | `min_erpm` | 4000.0 | ERPM | 0이 아닌 명령의 최소 ERPM | 측정 (vehicle_utils.MIN_DRIVING_RPM) | v_min = min_erpm / erpm_gain | 측정값 고정 |
| config/exp_common.yaml | 32 | `t0_speed_threshold` | 0.3 | m/s | 출발 시각 t0 판정 속도 | 기획안 | 출발 시각과 belief 초기화 시점 | 0.2~0.4 |
| config/exp_common.yaml | 33 | `hold_on_speed` | 0.35 | m/s | 최소 속도 hold 히스테리시스 켜짐 (v_E 초과) | 임시 | 낮추면 저속에서도 hold로 v_min을 낸다 (정지 직후 튀어나감) | hold_off_speed + 0.05 이상 |
| config/exp_common.yaml | 34 | `hold_off_speed` | 0.25 | m/s | 최소 속도 hold 히스테리시스 꺼짐 (v_E 미만) | 임시 | 높이면 감속 중 hold가 빨리 풀려 정지가 잦다 | 0.15~0.30 |
| config/exp_common.yaml | 36 | `d_follow_a` | 0.8 | m | 방법 a 고정 추종 거리 | 기획안 | 방법 a 추종 거리 | 0.6~1.0 |
| config/exp_common.yaml | 37 | `T_wait_a` | 3.0 | s | 방법 a 대기 시간 | 기획안 | 방법 a가 추월을 시도하기까지의 시간 | 2~5 |
| config/exp_common.yaml | 38 | `d_follow_b` | 1.5 | m | 방법 b 원거리 추종 거리 | 기획안 | d_y_max보다 커야 "정보 없음" 기준선이 된다 | > d_y_max |
| config/exp_common.yaml | 39 | `d_probe_c` | 0.4 | m | 방법 c 공격적 접근 거리 | 기획안 | 작을수록 AEB와 h < 0이 늘어난다 (기준선의 의도) | 0.3~0.5 |
| config/exp_common.yaml | 40 | `k_d` | 1.5 | 1/s | 거리 유지 P 게인 (방법 a·b·c) | 임시 | 키우면 거리 수렴이 빠르지만 속도 명령이 흔들린다 | 1.0~2.5 |
| config/exp_common.yaml | 41 | `probe_tol` | 0.05 | m | PROBE 진입 허용 오차 (d ≤ d_ref + 이 값) | 기획안 | 키우면 PROBE에 일찍 들어간다 (T_max·T_wait_a 시작 시점) | 0.03~0.10 |
| config/exp_common.yaml | 42 | `merge_tol` | 0.05 | m | MERGE 완료 판정 횡오차 | 임시 | 키우면 DONE 판정이 빨라진다 | 0.03~0.08 |
| config/exp_common.yaml | 43 | `track_hold` | 0.5 | s | 추적이 끊겨도 직전 값을 쓰는 시간 | 임시 | 키우면 추적 끊김에 둔해지지만 오래된 d로 속도를 낸다 | 0.2~0.8 |
| config/exp_common.yaml | 45 | `lidar_to_front` | 0.30 | m | 라이다 → 앞범퍼 (AEB lidar_to_bumper_dist와 같게) | 측정 (실측 0.28, 코드 0.30) | d 전체에 더해지는 오프셋. AEB와 다르면 d와 closest가 어긋난다 | 측정값 고정 |
| config/exp_common.yaml | 46 | `lidar_to_rear` | 0.22 | m | 라이다 → 뒤범퍼 | 측정 | d_rear와 MERGE 판정 | 측정값 고정 |
| config/exp_common.yaml | 47 | `cluster_jump` | 0.15 | m | 연속 점 사이 거리가 이보다 크면 클러스터 분할 | 임시 | 키우면 벽 근처 차와 벽 조각이 붙고, 줄이면 먼 차가 쪼개진다 | 0.10~0.20 |
| config/exp_common.yaml | 48 | `cluster_min_points` | 3 | 개 | 클러스터 최소 점 수 | 임시 | 키우면 먼 차를 놓치고, 줄이면 잡음 점을 차로 본다 | 3~6 |
| config/exp_common.yaml | 49 | `track_x_max` | 3.0 | m | 전방 차량 검출 최대 거리 | 임시 | 검출 시작 거리 | 2.5~4.0 |
| config/exp_common.yaml | 50 | `veh_width_min` | 0.12 | m | 차량 클러스터 폭 게이트 하한 | 임시 | 줄이면 벽 조각·기둥을 차로 본다 | 0.10~0.20 |
| config/exp_common.yaml | 51 | `veh_width_max` | 0.45 | m | 차량 클러스터 폭 게이트 상한 | 임시 | 선행차가 틀면 보이는 폭이 w·cosψ + L·sinψ로 커진다. 작으면 양보 중 추적이 끊긴다 | 0.45~0.60 |
| config/exp_common.yaml | 52 | `veh_depth_max` | 0.60 | m | 차량 클러스터 깊이(x 범위) 상한 | 임시 | 헤딩이 틀어졌을 때 통로 안으로 들어온 벽 조각 제거 | 0.55~0.70 |
| config/exp_common.yaml | 53 | `wall_margin` | 0.05 | m | 벽에서 이 거리 안의 점은 벽으로 버림 | 임시 | c_lat보다 작아야 벽에 붙은 차를 잃지 않는다. 너무 작으면 벽 잡음이 남는다 | 0.03~0.08 |
| config/exp_common.yaml | 54 | `rear_fov_half_deg` | 60.0 | deg | 후방 차량 검출 반시야각 | 임시 | 선행차가 후행차를 찾는 범위 | 45~75 |
| config/exp_common.yaml | 55 | `track_alpha` | 0.5 | - | d의 α-β 필터 α | 임시 | 키우면 d 추정이 빠르고 잡음에 민감하다 | 0.3~0.7 |
| config/exp_common.yaml | 56 | `track_beta` | 0.2 | - | d의 α-β 필터 β | 임시 | 키우면 v_B 추정이 빠르고 잡음에 민감하다 | 0.05~0.3 |
| config/exp_common.yaml | 57 | `track_jump_reset` | 0.3 | m | 예측과 이만큼 다르면 필터 초기화 | 임시 | 작으면 잡음에 필터가 초기화되어 v_B가 v_E로 튄다 | 0.2~0.5 |
| config/exp_common.yaml | 58 | `track_timeout` | 0.5 | s | 이 시간 넘게 안 보이면 필터 초기화 | 임시 | 끊긴 뒤 필터를 이어 쓸지 | 0.3~1.0 |
| config/exp_common.yaml | 59 | `side_gate` | 0.6 | m | 측면 추적 연관 거리 | 임시 | 추월 중 선행차를 다시 붙잡는 범위 | 0.4~0.8 |
| config/exp_common.yaml | 61 | `corridor_sector_min_deg` | 45.0 | deg | 벽 거리 측면 섹터 시작각 | 측정 (lidar_reactive_drive와 같은 값) | 벽 거리 섹터 | 기존 값 유지 |
| config/exp_common.yaml | 62 | `corridor_sector_max_deg` | 115.0 | deg | 벽 거리 측면 섹터 끝각 | 측정 (lidar_reactive_drive와 같은 값) | 벽 거리 섹터 | 기존 값 유지 |
| config/exp_common.yaml | 63 | `corridor_min_points` | 8 | 개 | 한쪽 벽으로 인정할 최소 점 수 | 측정 (lidar_reactive_drive와 같은 값) | 벽 인정 점 수 | 기존 값 유지 |
| config/exp_common.yaml | 64 | `corridor_wall_percentile` | 0.35 | - | 측면 거리 분위값 | 측정 (lidar_reactive_drive와 같은 값) | 벽 거리 분위값 | 기존 값 유지 |
| config/exp_common.yaml | 65 | `min_point_x` | -1.0 | m | 벽 거리에 쓰는 점의 x 하한 (앞범퍼 기준), 기존 주행은 −0.05 | 임시 | −0.05면 벽 거리 < 0.25 m에서 벽을 놓친다 (추월 차선 0.235 m). −1.0은 곡선에서 편향이 더 작지만 앞쪽 벽을 미리 보는 효과가 없다 | −1.0 (직선 통로) |
| config/exp_common.yaml | 67 | `d_brake_trigger` | 1.0 | m | NY 급정지 이벤트 무장 거리 | 기획안 | NY 급정지 이벤트가 무장되는 거리 | 0.8~1.2 |
| config/exp_common.yaml | 68 | `brake_delay_min` | 0.5 | s | 급정지 지연 최소 | 기획안 | 급정지 시점 난수 범위 | 0.5 |
| config/exp_common.yaml | 69 | `brake_delay_max` | 2.0 | s | 급정지 지연 최대 | 기획안 | 급정지 시점 난수 범위 | 2.0 |
| config/exp_common.yaml | 70 | `start_gap_margin` | 0.2 | m | t0에서 d_rear < d_y_max + 이 값이면 경고 | 기획안 | 출발 간격 경고 기준 | 0.2 |
