#!/usr/bin/env python3
"""회차 메타데이터 (record_run.sh가 호출). 경로 규칙은 my_algo/run_paths.py.

    run_meta.py start  <mode> <method> <scenario> <d_e|na> <run_id>   → run_dir 출력, meta.json 생성
    run_meta.py finish <run_dir>                                       → 종료 시각, 폐기 여부(abort.json) 합침
    run_meta.py discard <run_dir> <사유>                               → 수동 폐기 표시 (예: battery_low)

meta.json 필드: mode, method, scenario, d_e, run_id, 시작/종료 시각, 시작 배터리 전압,
    AEB 발동 여부·횟수·첫 발동 시각·위치·원인 점(aeb), S2–S4에서 정지선 전 발동 시 aeb_malfunction=true,
    git commit hash(+dirty), common.yaml sha256과 세션 첫 회차 일치 여부,
    ERPM_GAIN, MIN_DRIVING_RPM, V_MIN, VESC speed_to_erpm_gain, 폐기 여부·사유.
확인 전 이름은 환경변수로 바꾼다:
    VESC_STATE_TOPIC (기본 /sensors/core), VESC_VOLTAGE_FIELD (기본 state.voltage_input),
    VESC_ODOM_NODE (기본: 이름에 odom이 들어간 첫 노드)
"""

import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

from my_algo.metrics import aeb_malfunction
from my_algo.run_paths import run_dir, session_dir
from my_algo.speed_policy import V_MIN
from my_algo.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM


def share_path(*parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('my_algo'), *parts)


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def git_info():
    """설치된 패키지가 아니라 소스 레포의 커밋을 기록한다 (MY_ALGO_REPO 또는 이 스크립트 기준)."""
    repo = os.environ.get('MY_ALGO_REPO') or os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    try:
        head = subprocess.check_output(['git', '-C', repo, 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(['git', '-C', repo, 'status', '--porcelain'], text=True).strip())
        return {'git_hash': head, 'git_dirty': dirty, 'git_repo': repo}
    except (OSError, subprocess.CalledProcessError):
        return {'git_hash': None, 'git_dirty': None, 'git_repo': repo,
                'git_note': 'MY_ALGO_REPO 환경변수로 소스 레포 경로를 지정할 것'}


def read_voltage(timeout=3.0):
    topic = os.environ.get('VESC_STATE_TOPIC', '/sensors/core')
    field = os.environ.get('VESC_VOLTAGE_FIELD', 'state.voltage_input')
    try:
        import rclpy
        from rosidl_runtime_py.utilities import get_message
        rclpy.init()
        node = rclpy.create_node('run_meta_voltage')
        end = time.time() + 1.0
        types = {}
        while time.time() < end and topic not in types:
            types = dict(node.get_topic_names_and_types())
            time.sleep(0.1)
        if topic not in types:
            raise RuntimeError(f'{topic} 없음')
        got = []
        node.create_subscription(get_message(types[topic][0]), topic, got.append, 10)
        end = time.time() + timeout
        while not got and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        if not got:
            raise RuntimeError(f'{topic} 메시지 수신 없음')
        value = got[0]
        for part in field.split('.'):
            value = getattr(value, part)
        return {'battery_voltage': float(value), 'battery_source': f'{topic}:{field}'}
    except Exception as exc:  # 기록 실패가 회차를 막지 않게 한다
        return {'battery_voltage': None, 'battery_source': f'{topic}:{field}', 'battery_error': str(exc)}


def vesc_gain():
    node = os.environ.get('VESC_ODOM_NODE')
    try:
        if not node:
            nodes = subprocess.check_output(['ros2', 'node', 'list'], text=True, timeout=10).split()
            node = next((n for n in nodes if 'odom' in n), None)
        if not node:
            return {'vesc_speed_to_erpm_gain': None, 'vesc_gain_note': 'odom 노드 못 찾음'}
        out = subprocess.check_output(['ros2', 'param', 'get', node, 'speed_to_erpm_gain'],
                                      text=True, timeout=10)
        return {'vesc_speed_to_erpm_gain': float(out.strip().split()[-1]), 'vesc_odom_node': node}
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        return {'vesc_speed_to_erpm_gain': None, 'vesc_odom_node': node, 'vesc_gain_note': str(exc)}


def write_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def start(mode, method, scenario, d_e, run_id):
    d_e = None if d_e in ('na', '') else float(d_e)
    rdir = run_dir(scenario, method, d_e, int(run_id))
    if os.path.exists(os.path.join(rdir, 'bag')):
        sys.exit(f'이미 기록된 회차: {rdir} (run_id를 바꿀 것)')
    os.makedirs(os.path.join(rdir, 'camera'), exist_ok=True)
    # 기록 시점 설정 사본. 분석은 이 사본을 읽어 나중에 config가 바뀌어도 재현된다.
    snap = os.path.join(rdir, 'config_snapshot')
    os.makedirs(snap, exist_ok=True)
    for name in ('common.yaml', f'method_{method}.yaml', f'scenario_{scenario}.yaml',
                 'vehicle_geometry.yaml', 'imu.yaml'):
        shutil.copy2(share_path('config', name), snap)

    common = share_path('config', 'common.yaml')
    common_hash = sha256(common)
    session_path = os.path.join(session_dir(), 'session.json')
    session = {}
    if os.path.exists(session_path):
        with open(session_path, encoding='utf-8') as f:
            session = json.load(f)
    if 'common_yaml_sha256' not in session:
        session['common_yaml_sha256'] = common_hash
        session['first_run'] = os.path.basename(rdir)
        write_json(session_path, session)
    matches = session['common_yaml_sha256'] == common_hash
    if not matches:
        print(f'경고: common.yaml이 세션 첫 회차({session.get("first_run")})와 다르다. '
              '방법 간 비교가 무효가 될 수 있음', file=sys.stderr)

    meta = {
        'mode': mode, 'method': method, 'scenario': scenario, 'd_e': d_e, 'run_id': int(run_id),
        'run_dir': rdir, 'start_time': datetime.datetime.now().isoformat(timespec='milliseconds'),
        'common_yaml_sha256': common_hash, 'common_yaml_matches_session': matches,
        'erpm_gain': ERPM_GAIN, 'min_driving_rpm': MIN_DRIVING_RPM, 'v_min': V_MIN,
        'discard': False, 'discard_reason': None,
        'odom_topic': os.environ.get('ODOM_TOPIC', '/vesc/odom'),
        'test_v_ref': float(os.environ['TEST_V_REF']) if os.environ.get('TEST_V_REF') else None,
        'test_step_v': float(os.environ['TEST_STEP_V']) if os.environ.get('TEST_STEP_V') else None,
        'cloud_topic': os.environ.get('CLOUD_TOPIC', '/livox/lidar'),
    }
    meta.update(git_info())
    meta.update(read_voltage())
    meta.update(vesc_gain())
    if meta['vesc_speed_to_erpm_gain'] is not None and abs(meta['vesc_speed_to_erpm_gain'] - ERPM_GAIN) > 1e-6:
        print(f'참고: 명령 ERPM_GAIN={ERPM_GAIN}과 odom speed_to_erpm_gain='
              f'{meta["vesc_speed_to_erpm_gain"]}이 다르다 (docs/OPEN_ITEMS.md)', file=sys.stderr)
    write_json(os.path.join(rdir, 'meta.json'), meta)
    print(rdir)


def scenario_stop_distance(scenario):
    """설치된 scenario yaml의 stop_distance (없으면 inf: 모든 발동을 정지선 전으로 본다)."""
    try:
        import yaml
        with open(share_path('config', f'scenario_{scenario}.yaml'), encoding='utf-8') as f:
            return float(yaml.safe_load(f)['/**']['ros__parameters']['stop_distance'])
    except (OSError, KeyError, ImportError, TypeError, ValueError):
        return float('inf')


def finish(rdir):
    path = os.path.join(rdir, 'meta.json')
    with open(path, encoding='utf-8') as f:
        meta = json.load(f)
    meta['end_time'] = datetime.datetime.now().isoformat(timespec='milliseconds')
    abort = os.path.join(rdir, 'abort.json')
    if os.path.exists(abort):
        with open(abort, encoding='utf-8') as f:
            info = json.load(f)
        meta['discard'] = True
        meta['discard_reason'] = info.get('reason')
    events = os.path.join(rdir, 'mux_events.jsonl')
    if os.path.exists(events):
        with open(events, encoding='utf-8') as f:
            meta['mux_events'] = [json.loads(line) for line in f if line.strip()]
    aeb_path = os.path.join(rdir, 'aeb_events.jsonl')
    aeb_events = []
    if os.path.exists(aeb_path):
        with open(aeb_path, encoding='utf-8') as f:
            aeb_events = [json.loads(line) for line in f if line.strip()]
    in_run = [e for e in aeb_events if e.get('auto')]
    first = in_run[0] if in_run else None
    meta['aeb'] = {'triggered': bool(in_run), 'count': len(in_run),
                   'first_time': None if first is None else first['ros_time'],
                   'first_travel_m': None if first is None else first['travel_m'],
                   'first_cause_point': None if first is None else first['cause_point'],
                   'events': aeb_events}
    # 돌출이 없는 시나리오에서 정지선 전에 AEB가 걸리면 오작동 (analysis/extract_run.py·시뮬과 같은 함수).
    # 효율 지표에서 빼고 방법별로 따로 센다. 최종 판정은 extract_run이 bag으로 다시 한다
    meta['aeb_malfunction'] = aeb_malfunction(meta['scenario'], meta['aeb']['first_travel_m'],
                                              scenario_stop_distance(meta['scenario']))
    bag = os.path.join(rdir, 'bag')
    meta['bag_bytes'] = sum(os.path.getsize(os.path.join(dp, fn))
                            for dp, _, fns in os.walk(bag) for fn in fns) if os.path.isdir(bag) else 0
    write_json(path, meta)
    print(f'{rdir}: discard={meta["discard"]} reason={meta["discard_reason"]} '
          f'aeb={meta["aeb"]["count"]} malfunction={meta["aeb_malfunction"]} '
          f'bag={meta["bag_bytes"] / 1e6:.1f} MB')


def discard(rdir, reason):
    path = os.path.join(rdir, 'meta.json')
    with open(path, encoding='utf-8') as f:
        meta = json.load(f)
    meta['discard'] = True
    meta['discard_reason'] = reason
    write_json(path, meta)
    print(f'{rdir}: 폐기 표시 ({reason})')


def main():
    if len(sys.argv) >= 7 and sys.argv[1] == 'start':
        start(*sys.argv[2:7])
    elif len(sys.argv) == 3 and sys.argv[1] == 'finish':
        finish(sys.argv[2])
    elif len(sys.argv) == 4 and sys.argv[1] == 'discard':
        discard(sys.argv[2], sys.argv[3])
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
