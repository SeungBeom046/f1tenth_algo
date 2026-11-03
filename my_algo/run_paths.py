"""회차 로그 경로 규칙 (bringup.launch.py와 scripts/run_meta.py가 같이 쓴다).

    ~/exp_logs/YYYYMMDD/<scenario>_<method>_de<d_e>_run<NN>/
        bag/  meta.json  params.yaml  camera/  abort.json(폐기 시)  mux_events.jsonl
    ~/exp_logs/YYYYMMDD/session.json
"""

import datetime
import os

METHODS = ('B0', 'B1', 'B2', 'P')
SCENARIOS = ('S1', 'S2', 'S3', 'S4')


def log_root():
    return os.path.expanduser(os.environ.get('EXP_LOG_ROOT', '~/exp_logs'))


def session_dir(date=None):
    date = date or os.environ.get('EXP_DATE') or datetime.date.today().strftime('%Y%m%d')
    return os.path.join(log_root(), date)


def format_de(d_e):
    """d_e 표기: 1.0 -> '1.0', 돌출 없는 시나리오는 'na'."""
    if d_e in (None, '', 'na', 'none'):
        return 'na'
    return f'{float(d_e):.1f}'


def run_name(scenario, method, d_e, run_id):
    return f'{scenario}_{method}_de{format_de(d_e)}_run{int(run_id):02d}'


def run_dir(scenario, method, d_e, run_id, date=None):
    if method not in METHODS:
        raise ValueError(f'method must be one of {METHODS}, got {method!r}')
    if scenario not in SCENARIOS:
        raise ValueError(f'scenario must be one of {SCENARIOS}, got {scenario!r}')
    return os.path.join(session_dir(date), run_name(scenario, method, d_e, run_id))
