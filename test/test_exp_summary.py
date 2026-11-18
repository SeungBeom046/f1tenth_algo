"""exp_planner의 인식 태그와 종료 요약 (rclpy 없이: 모듈 import에 ROS가 필요해 함수만 떼어 검사한다)."""
import ast
import collections
import json
import math
import os
import time
from dataclasses import replace

from my_algo.aur.ego_runner import EgoRunner
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES

SRC = os.path.join(os.path.dirname(__file__), '..', 'my_algo', 'exp_planner.py')


def _load():
    tree = ast.parse(open(SRC).read())
    keep = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
            and n.name in ('perception_tags', 'RunStats')]
    ns = {'collections': collections, 'json': json, 'math': math, 'time': time}
    exec(compile(ast.Module(body=keep, type_ignores=[]), SRC, 'exec'), ns)
    return ns['perception_tags'], ns['RunStats']


def test_tags_and_summary():
    perception_tags, RunStats = _load()
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
