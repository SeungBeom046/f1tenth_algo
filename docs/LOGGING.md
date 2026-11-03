# 로그 저장 위치와 구조

구현: 경로 규칙 [my_algo/run_paths.py](../my_algo/run_paths.py), 기록 [scripts/record_run.sh](../scripts/record_run.sh),
메타데이터 [scripts/run_meta.py](../scripts/run_meta.py). 루트는 `EXP_LOG_ROOT`(기본 `~/exp_logs`), 날짜는 `EXP_DATE`로 바꿀 수 있다.

## 1. 경로 규칙

```
~/exp_logs/YYYYMMDD/<scenario>_<method>_de<d_e>_run<NN>/     예: S1_P_de1.5_run03, S3_B2_dena_run01, E0_B0_dena_run02
  bag/                 rosbag2 (sqlite3 `*.db3`, Foxy 기본). 분석은 rosbags 0.9.23
  meta.json            회차 메타데이터 (아래 2절)
  params.yaml          실제 로드된 파라미터 전체 덤프 (실행 중 exp 노드별 `ros2 param dump --print`)
  config_snapshot/     기록 시점 common / method / scenario / vehicle_geometry / imu YAML 사본 (분석이 이것을 읽음)
  camera/              천장 카메라 파일(수동 복사, ceiling.mp4), 오프라인 AEB 오버레이 결과
  aeb_events.jsonl     AEB 발동 순간 이벤트 (exp_aeb_node)
  mux_events.jsonl     정지 이벤트 (joy_timeout, foreign_publisher, stop_line, aeb)
  abort.json           폐기 사유 (조이스틱 끊김, 예상 외 발행자) — 있으면 meta.json discard=true
  analysis/            extract_run.py 결과 (metrics.json, speed_vs_s.csv, ceiling.json, offline_compare.json)
  replay/<method>/     offline_replay.sh 결과 bag (/clock 포함 — Foxy record에 --use-sim-time이 없어 분석에서 bag 시각으로 변환)
~/exp_logs/YYYYMMDD/session.json     세션 정보 (아래 4절)
~/exp_logs/analysis/                 summarize.py, plots.py 결과 (summary.csv/md, 그림)
~/exp_logs/popout/popout_events.jsonl   #2 Jetson의 #2 기록 (트리거 수신 지연, 도달 시간, 속도)
```

d_e 표기: S1은 `1.0`/`1.5`/`2.0`, 그 밖은 `na`. run_id는 블록 번호(`randomize_runs.py`).

## 2. meta.json

| 키 | 내용 | 시점 |
|---|---|---|
| mode, method, scenario, d_e, run_id, run_dir | 회차 조건 | start |
| start_time, end_time | ISO 시각 | start / finish |
| battery_voltage, battery_source | 시작 배터리 전압 (`VESC_STATE_TOPIC`:`VESC_VOLTAGE_FIELD`, TBD(세션 1)) | start |
| git_hash, git_dirty, git_repo | 소스 레포 커밋 (`MY_ALGO_REPO`) | start |
| common_yaml_sha256, common_yaml_matches_session | common.yaml 해시와 세션 첫 회차 일치 여부 | start |
| erpm_gain, min_driving_rpm, v_min | 명령 변환 (`vehicle_utils.py`) | start |
| vesc_speed_to_erpm_gain, vesc_odom_node | odom 변환 (VESC 설정, TBD(세션 1)) | start |
| odom_topic, cloud_topic | 기록에 쓴 토픽 이름 | start |
| discard, discard_reason | 폐기 여부: 조이스틱 끊김 `joy_timeout`, 예상 외 발행자 `foreign_publisher`, 수동 사유 | finish / discard |
| aeb.triggered, aeb.count, aeb.first_time, aeb.first_travel_m, aeb.first_cause_point, aeb.events | AEB 발동 기록 (원인 점 = 가장 가까운 점, 앞 끝 기준 x, y) | finish |
| aeb_malfunction | S2–S4에서 AEB 발동 = 오작동 (효율 통계에서 제외, 방법별로 따로 집계) | finish |
| mux_events | 정지 이벤트 목록 | finish |
| bag_bytes | bag 크기 | finish |

`v_ref` 미도달 폐기는 분석 단계(`extract_run.py`)에서 `metrics.json`에 `discard_reason: vref_not_reached`로 붙는다.

### common.yaml 해시 경고
첫 회차의 `common.yaml` sha256을 `session.json`에 저장하고, 이후 회차가 다르면 `record_run.sh` 시작 시
`경고: common.yaml이 세션 첫 회차(...)와 다르다`를 출력하고 `common_yaml_matches_session: false`를 남긴다.
통제변인이 세션 중에 바뀌었다는 뜻이므로 그 회차들을 따로 보거나 다시 한다.

## 3. 기록 토픽과 용도

| 토픽 | 용도 (지표) |
|---|---|
| 점군 `CLOUD_TOPIC` (TBD(세션 1)) | 오프라인 재생, RQ4 재계산 |
| odom `ODOM_TOPIC` | 위치 s, 실측 속도 → 실측 기준 손실, 최저 속도, 감속 구간, v_ref 도달, 트리거 확인 |
| `/livox/imu`(원본), `/imu`, `/imu/linear_accel_base`, `/imu/status` | jerk RMS, Exp0 a_b·t_r |
| VESC 상태 `VESC_STATE_TOPIC` (TBD(세션 1)) | 속도·전압 확인 |
| `/commands/motor/speed`, `/commands/servo/position`, `/exp/final_cmd` | 실제 보낸 명령 |
| `/exp/v_raw` | **명령 기준 손실** (min(v_ref, v_cap), 정책 전) — 논문 주장의 핵심 근거 |
| `/exp/v_cap`, `/exp/v_cmd`, `/exp/min_speed_flag` | 방법별 상한, 정책 후 명령, V_MIN 유지 비율 |
| `/exp/phantom_d`, `/exp/rho`, `/exp/phantoms`, `/exp/phantom_debug` | d, ρ, phantom 위치, P 열·틈, phantom 깜빡임 (RQ4) |
| `/exp/grid`, `/exp/boundary`, `/exp/boundary_frame`, `/exp/grid_debug` | 격자, 경계, 프레임별 경계 흔들림 (RQ4), 처리 시간 |
| `/exp/steering_command`, `/scan` | 조향, 최소 거리·TTC (footprint 기준) |
| `/aeb/active`, `/aeb/status`, `/exp/aeb_event` | AEB 발동, 원인, 오작동 판정 |
| `/exp/stop_reason`, `/exp/abort` | 정지 사유, 폐기 사유 |
| `/trigger`, `/trigger/info`, `/popout/status` | 트리거 시각, 트리거 거리, #2 수신 지연·도달 시간 |
| `/autonomous_mode`, `/joy_active`, `/joy` | AUTO 시작 시각 (s = 0 기준), 비상 정지 |
| `/tf`, `/tf_static` | 재생, 장착 오프셋 |
| 압축 전방 카메라 `CAMERA_TOPIC` (TBD(세션 1)) | 오프라인 AEB 오버레이 |

`EXTRA_TOPICS="..."`로 추가할 수 있다. drive 모드 기록이면 `/reactive/lidar_command`도 들어간다.

## 4. session.json

| 키 | 쓰는 곳 |
|---|---|
| common_yaml_sha256, first_run | `run_meta.py start` (첫 회차) |
| run_order {seed, blocks, runs[]} | `randomize_runs.py` |
| imu_calib, time_sync | 수동 (RUNBOOK 0.3, 0.4절 결과를 붙인다) |

## 5. 폐기 회차 표시

| 사유 | 표시 주체 | 위치 |
|---|---|---|
| 조이스틱 끊김 `joy_timeout` | exp_drive_mux (자동) | `abort.json` → `meta.json` |
| 예상 외 발행자 `foreign_publisher` | exp_drive_mux (자동) | `abort.json` → `meta.json` |
| v_ref 미도달 `vref_not_reached` | extract_run.py (자동) | `analysis/metrics.json` |
| 배터리, 트랙 어긋남 등 | 사람: `python3 $S/run_meta.py discard <run_dir> <사유>` | `meta.json` |
| AEB 오작동 (S2–S4) | 자동 | `meta.json` aeb_malfunction — 폐기가 아니라 효율 통계 제외 + 별도 집계 |

폐기 회차도 지우지 않는다. `summarize.py`가 폐기 수와 사유를 표에 함께 낸다.

## 6. 디스크 용량 추정

회차당 기록 약 90초 기준 (TBD(세션 1): 실제 점군 크기로 갱신):

| 토픽 | 추정 |
|---|---|
| Mid-360 점군 (약 2만 점 × 약 26 B × 10 Hz) | 약 5 MB/s |
| 압축 전방 카메라 (640×480 JPEG 30 fps) | 약 1.5 MB/s |
| 격자·경계·기타 | 약 0.5 MB/s |
| 합계 | 약 7 MB/s × 90 s ≈ **0.6 GB/회** |

235회 ≈ **150 GB** (+ Exp0·Exp1). 하루 분량(약 80회 ≈ 50 GB)마다 외장 디스크로 옮긴다.
실제 크기는 `meta.json`의 `bag_bytes`로 첫 회차 뒤 다시 계산한다.
