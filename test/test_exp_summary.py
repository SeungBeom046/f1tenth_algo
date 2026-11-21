"""exp_planner의 인식 태그와 종료 요약 (rclpy 없이: 모듈 import에 ROS가 필요해 함수만 떼어 검사한다)."""
import ast
import collections
import json
import math
import os
import time
from dataclasses import replace
from types import SimpleNamespace

from my_algo.aur.camera import fused_scan
from my_algo.aur.ego_runner import EgoRunner
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES

SRC = os.path.join(os.path.dirname(__file__), '..', 'my_algo', 'exp_planner.py')


def _load():
    tree = ast.parse(open(SRC).read())
    keep = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
            and n.name in ('perception_tags', 'RunStats', 'camera_inputs', 'start_kick')]
    ns = {'collections': collections, 'json': json, 'math': math, 'time': time,
          'SimpleNamespace': SimpleNamespace, 'fused_scan': fused_scan}
    exec(compile(ast.Module(body=keep, type_ignores=[]), SRC, 'exec'), ns)
    return ns['perception_tags'], ns['RunStats'], ns['camera_inputs'], ns['start_kick']


def test_tags_and_summary():
    perception_tags, RunStats, _, _ = _load()
    from _aur_synth import scan
    from test_aur_guardrail import scan_message
    runner = EgoRunner(replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0)), 'd')
    stats = RunStats()
    msg = scan_message(scan(1., 0., n=1440))
    tags = []
    for k in range(5):
        runner.step(k * .1, msg, 0.)
        rail, lead = perception_tags(runner)
        tags.append(rail)
        stats.scan(k * .1, 0.5, 0.0, 5.0, {'state': runner.fsm.state}, 'corridor_invalid', rail, lead.split()[0])
    assert tags[0] in ('rail_lost', 'rail_init') and all(t.startswith('rail_') for t in tags)
    stats.aeb('ttc=0.71s closest=0.64m')
    text = stats.summary('ego')
    assert '레일 인식' in text and 'AEB 작동             : 1회' in text and '0.20 m' in text
    assert '카메라               : 꺼짐' in text
    stats.camera('cam_ok', True)
    assert 'YOLO 앞차 확인 1 scan' in stats.summary('ego')


def test_camera_inputs_fuse_only_fresh_results():
    camera_inputs = _load()[2]
    msg = SimpleNamespace(ranges=[2.0, 2.0, 2.0], angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=20.0)
    cam = SimpleNamespace(ranges=[float('inf'), 1.0, float('inf')], angle_min=0.0, angle_increment=0.1)
    lead = {'r': 1.2, 'a_lo': -0.1, 'a_hi': 0.1}
    on = replace(AurParams(), camera_fusion=True, camera_lead=True)
    scan, got, tag = camera_inputs(on, 1.0, msg, (0.9, cam), (0.95, lead))
    assert list(scan.ranges) == [2.0, 1.0, 2.0] and got == lead and tag == 'cam_ok'
    scan, got, tag = camera_inputs(on, 2.0, msg, (0.9, cam), (0.95, lead))     # 1 s 지남: 라이다만
    assert scan is msg and got is None and tag == 'cam_none'
    assert camera_inputs(AurParams(), 1.0, msg, (0.9, cam), (0.95, lead)) == (msg, None, 'cam_off')
    # camera_lead만: YOLO 앞차 범위(방위 −0.1~0.1, 거리 1.1~1.8 m) 안 bin만 합친다
    only = replace(AurParams(), camera_lead=True)
    near = SimpleNamespace(ranges=[1.0, 1.2, 1.9], angle_min=-0.15, angle_increment=0.1)
    msg3 = SimpleNamespace(ranges=[2.0, 2.0, 2.0], angle_min=-0.15, angle_increment=0.1, range_min=0.05, range_max=20.0)
    scan, _, _ = camera_inputs(only, 1.0, msg3, (0.9, near), (0.95, lead))
    assert list(scan.ranges) == [2.0, 1.2, 2.0]     # 1.0은 r − 0.1 = 1.1보다 가깝고 1.9는 r + 0.6 = 1.8보다 멀다
    scan, _, _ = camera_inputs(only, 1.0, msg3, (0.9, near), None)
    assert scan is msg3                             # 앞차가 없으면 카메라 거리를 안 합친다


def test_start_kick_only_while_commanded_and_stalled():
    kick = _load()[3]
    st = {}
    # 정지(측정 0.07 m/s 잡음)에서 출발 명령: 0.5 s 켜고 0.5 s 쉬기 반복
    on = [kick(st, 0.1 * k, 0.87, 0.07, 5500.0, 0.5) for k in range(12)]
    assert on == [True] * 5 + [False] * 5 + [True] * 2
    assert not kick(st, 1.2, 0.87, 0.3, 5500.0, 0.5)              # 움직이면 끝
    assert kick(st, 1.3, 0.87, 0.05, 5500.0, 0.5)                 # 다시 서면 새로 시작
    assert not kick(st, 1.4, 0.0, 0.05, 5500.0, 0.5)              # 정지 명령이면 안 함
    assert not kick({}, 0.0, 0.87, 0.0, 0.0, 0.5)                 # 0 = 끔
