# analysis/

ROS가 있는 PC(또는 Jetson)에서 `source install/setup.bash` 후 실행한다 (bag 읽기에 rosbag2_py 필요).
그림은 matplotlib, 천장 카메라·오버레이는 OpenCV(aruco 포함), 한글 라벨은 `fonts-nanum`.

| 순서 | 명령 | 결과 |
|---|---|---|
| 1 | `python3 analysis/extract_run.py --all ~/exp_logs/<날짜>` | 회차별 `analysis/metrics.json`, `speed_vs_s.csv` |
| 2 | `python3 analysis/ceiling_camera.py --video ... --calib ... --run-dir ...` | 통과시각, 실제 d_e, 최소 거리 (`ceiling.json`) |
| 3 | `python3 analysis/summarize.py ~/exp_logs/<날짜> ...` | `~/exp_logs/analysis/summary.{csv,md}` (시나리오별, 평균 ± 95% CI, 폐기 수) |
| 4 | `python3 analysis/plots.py ~/exp_logs/<날짜> ...` | 그림 5, 그림 6, RQ4 |
| 선택 | `bash scripts/offline_replay.sh <run_dir> B1 B2 P` → `python3 analysis/offline_compare.py <run_dir>` | 같은 관측에서 방법별 명령 비교 (개루프) |
| 선택 | `python3 analysis/aeb_overlay.py <run_dir>` | 전방 영상에 AEB 상태 오버레이 |

지표 정의와 고정값(jerk 저역통과 5 Hz 등)은 `my_algo/metrics.py` 맨 위 주석.
