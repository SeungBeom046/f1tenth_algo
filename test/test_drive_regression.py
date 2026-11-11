"""rclpy 없이 도는 주행 출력 회귀 테스트.

scripts/verify_refactor_offline.py로 리팩터링 전 main(b054758)과 현재 작업 트리의
lidar_reactive_drive·autonomous_drive 발행 메시지를 합성 입력에서 비교한다.
"""
import os
import subprocess
import sys
import types

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = 'b054758'


def _has_rev(rev):
    try:
        subprocess.check_output(['git', '-C', REPO, 'cat-file', '-e', rev + '^{commit}'],
                                stderr=subprocess.DEVNULL)
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


@pytest.mark.skipif(not _has_rev(BASELINE), reason='git 이력 없음 (기준선 커밋을 찾을 수 없음)')
def test_drive_outputs_match_pre_refactor_main():
    res = subprocess.run(
        [sys.executable, os.path.join(REPO, 'scripts', 'verify_refactor_offline.py'),
         '--baseline', BASELINE, '--seeds', '3'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    assert res.returncode == 0, res.stdout


def test_min_speed_rule_unchanged():
    # 합성 입력은 최소 속도 클램프 구간을 거의 지나지 않아 회귀 비교로는 못 잡는다. 값을 직접 고정한다.
    sys.path.insert(0, REPO)
    from my_algo import vehicle_utils as vu

    assert (vu.MIN_DRIVING_RPM, vu.CRUISE_RPM, vu.MAX_RPM, vu.ERPM_GAIN) == (4000.0, 5740.0, 12000.0, 4614.0)
    assert vu.sanitize_rpm(0.0) == 0.0
    assert vu.sanitize_rpm(1.0) == 4000.0
    assert vu.sanitize_rpm(-1.0) == -4000.0
    assert vu.sanitize_rpm(5000.0) == 5000.0
    assert vu.sanitize_rpm(1e9) == 12000.0
    assert vu.sanitize_rpm(float('nan')) == 0.0


def test_log_effective_parameters_prints_every_value():
    sys.path.insert(0, REPO)
    from my_algo.vehicle_utils import log_effective_parameters

    lines = []
    node = types.SimpleNamespace(
        _parameters={'b': types.SimpleNamespace(value=2.0), 'a': types.SimpleNamespace(value=True)},
        get_logger=lambda: types.SimpleNamespace(info=lines.append),
    )
    log_effective_parameters(node)
    assert lines == ['effective parameters: a=True, b=2.0']
