# analysis/

**노트북(분석 PC)에서 실행한다.** ROS 설치가 필요 없다 (bag은 rosbags 0.9.23으로 읽는다, Foxy sqlite3 bag).

```bash
python3 -m venv ~/venv_exp && source ~/venv_exp/bin/activate
pip install -r analysis/requirements.txt      # rosbags==0.9.23 (Python 3.8–3.11; 0.10+는 3.10 이상 필요)
cd <my_algo 레포> && export PYTHONPATH=.       # my_algo.metrics 등 순수 모듈을 쓴다
```
그림 한글 라벨: 맥은 AppleGothic 자동, 우분투는 `sudo apt install fonts-nanum`.

| 실행 위치 | 스크립트 |
|---|---|
| 노트북 | `analysis/*.py` 전부 (extract_run, summarize, plots, ceiling_camera, aeb_overlay, exp0_braking, odom_travel, offline_compare) |
| 에고 Jetson (Foxy, Python 3.8) | `scripts/offline_replay.sh` (노드 재실행이 필요해서). 결과 비교 `offline_compare.py`는 노트북 |
| 둘 다 가능 | Jetson에서 돌리려면 `pip3 install --user rosbags==0.9.23` (Ubuntu 20.04 numpy 1.17 그대로) |

| 순서 | 명령 | 결과 |
|---|---|---|
| 1 | `python3 analysis/extract_run.py --all ~/exp_logs/<날짜>` | 회차별 `analysis/metrics.json`, `speed_vs_s.csv` |
| 2 | `python3 analysis/ceiling_camera.py --video ... --calib ... --run-dir ...` | 통과시각, 실제 d_e, 최소 거리 (`ceiling.json`) |
| 3 | `python3 analysis/summarize.py ~/exp_logs/<날짜> ...` | `~/exp_logs/analysis/summary.{csv,md}` (시나리오별, 평균 ± 95% CI, 폐기 수) |
| 4 | `python3 analysis/plots.py ~/exp_logs/<날짜> ...` | 그림 5, 그림 6, RQ4 |
| 선택 | `bash scripts/offline_replay.sh <run_dir> B1 B2 P` → `python3 analysis/offline_compare.py <run_dir>` | 같은 관측에서 방법별 명령 비교 (개루프) |
| 선택 | `python3 analysis/aeb_overlay.py <run_dir>` | 전방 영상에 AEB 상태 오버레이 |

지표 정의와 고정값(jerk 저역통과 5 Hz 등)은 `my_algo/metrics.py` 맨 위 주석.
