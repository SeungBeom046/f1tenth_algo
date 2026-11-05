#!/usr/bin/env bash
# sim/.venv 생성·설치·점검. 다시 실행해도 안전하다 (있으면 재사용, 버전만 맞춘다).
#   bash sim/setup_env.sh
# 시스템 Python·Homebrew·conda 패키지는 건드리지 않는다. venv 기반은 Homebrew python3.12 (arm64).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/.venv"
BASE_PY="${SIM_BASE_PYTHON:-/opt/homebrew/bin/python3.12}"

if [ "$(uname -m)" != arm64 ]; then echo "경고: arm64가 아니다 ($(uname -m)). Rosetta 터미널이면 네이티브 터미널로 다시." >&2; fi
if [ ! -x "$VENV/bin/python" ]; then
  [ -x "$BASE_PY" ] || { echo "$BASE_PY 없음: brew install python@3.12 또는 SIM_BASE_PYTHON=<python3.10+>" >&2; exit 1; }
  echo "venv 생성: $VENV ($("$BASE_PY" --version))"
  "$BASE_PY" -m venv "$VENV"
fi
"$VENV/bin/python" -m pip install -q --upgrade pip
"$VENV/bin/pip" install -q -r "$HERE/requirements.txt"
"$VENV/bin/python" "$HERE/check_env.py"
