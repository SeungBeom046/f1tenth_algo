#!/usr/bin/env python3
"""시뮬 로그(sim_logs/.../log.npz, meta.json) → 실차와 같은 지표 (<run_dir>/analysis/metrics.json, speed_vs_s.csv).

지표 계산은 my_algo/metrics.py 함수를 그대로 쓴다 (extract_run.py와 같은 함수, 복제 없음). 실차와 다른 점:
- 위치 s = AUTO 이후 에고 주행 거리 (시뮬 참값). 실측 속도 = 차량 모델 속도.
- 최소 거리·충돌: 에고 footprint와 #2 다각형 사이 거리(geometry.polygon_distance) — 실차의 천장 카메라 방식과 같음.
- jerk: 차량 모델 속도를 미분한 종방향 가속도 → metrics.jerk_rms (저역통과 5 Hz 같음).
- RQ4: 프레임별 가림 경계 → metrics.boundary_jitter, phantom 수 → metrics.phantom_flicker.

    sim/.venv/bin/python analysis/sim_reader.py <sim run_dir> [...]
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from my_algo import metrics as M  # noqa: E402
from my_algo.braking import step_lag, tracking_lag  # noqa: E402
from my_algo.track_layout import from_params as layout_from_params  # noqa: E402


def _front_offset(meta):
    # start_pose x = -rear_axle_to_front (앞 끝이 트랙 x = 0에서 출발)
    return -float(meta['start_pose'][0])


def _enter_delay(meta, t_enter):
    trig = [e['t'] for e in meta.get('events', []) if e['topic'] == '/trigger']
    return (t_enter - trig[0]) if trig else None


def compute(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    z = np.load(os.path.join(run_dir, 'log.npz'))
    cfg = meta['config']
    scen, v_ref = cfg['scenario_params'], float(cfg['common']['v_ref'])
    auto = z['t'] >= float(cfg['sim']['sim']['auto_start_time'])
    t, s, v = z['t'][auto], z['s'][auto], z['v'][auto]
    x0, x1 = float(scen['test_x_start']), float(scen['test_x_end'])
    lay = layout_from_params(scen)
    r0, r1 = lay.row_x_start, lay.row_x_end
    out = {'run': os.path.basename(run_dir), 'method': meta['method'], 'scenario': meta['scenario'],
           'd_e': meta['d_e'], 'run_id': meta['run_id'], 'seed': meta['seed'], 'source': 'sim',
           'discard': False, 'discard_reason': None}
    series = {'cmd': z['v_raw'][auto], 'meas': v, 'final': z['final_cmd'][auto]}
    grids = {}
    for tag, vv in series.items():
        ok = np.isfinite(vv)
        ss, vs = M.series_on_s(t, s, t[ok], vv[ok]) if ok.any() else (np.empty(0), np.empty(0))
        grids[tag] = (ss, vs)
        dt, floored = M.loss_delta_t(ss, vs, v_ref, x0, x1)
        out[f'dT_{tag}'] = dt
        out[f'dT_{tag}_floored'] = floored
        out[f'window_loss_pct_{tag}'] = M.window_loss_pct(ss, vs, v_ref, r0, r1) if lay.boxes else None
        out[f'min_speed_{tag}'] = M.min_speed(ss, vs, x0, x1)
        out[f'decel_length_{tag}'] = M.decel_length(ss, vs, v_ref, x0, x1)
    out['vref_reached'] = M.vref_reached(*grids['meas'], float(scen['accel_end_x']), v_ref)
    # 폐기: 실차와 같은 함수 (가속 구간 끝 명령 = v_ref인데 실측 미달일 때만)
    bad, why, info = M.discard_check(*grids['final'], *grids['meas'], float(scen['accel_end_x']), v_ref)
    out.update(info)
    if bad:
        out['discard'], out['discard_reason'] = True, why
    in_test = (s >= x0) & (s <= x1)
    a = np.gradient(v, t) if len(t) > 2 else np.zeros_like(v)
    out['jerk_rms'] = M.jerk_rms(t, a, float(t[in_test][0]), float(t[in_test][-1])) if in_test.any() else None
    out['jerk_cutoff_hz'] = M.JERK_CUTOFF_HZ
    dist = z['dist_car2'][auto]
    if np.isfinite(dist).any():
        out.update(M.safety_metrics(t, dist, v, collision_tol=0.0))
    else:
        out.update({'min_distance': None, 'collision': False, 'impact_speed': None, 'min_ttc': None,
                    'min_ego_ttc': None})
    # 추종 지연 (Exp0 i와 같은 함수): 명령이 v_cap을 따라 내려가는 구간의 등가 지연. 시뮬 t_r_eff 가정 확인용
    out['tracking_lag'] = None
    if meta['method'] in ('B2', 'P'):
        r = tracking_lag(t, z['final_cmd'][auto], t, v, float(cfg['common']['a_b']))
        if r and r['n'] >= 20:
            out['tracking_lag'], out['tracking_lag_dead'], out['tracking_lag_tau'] = r['lag'], r['dead'], r['tau']
    if (cfg.get('run_extra') or {}).get('test_step_v'):
        # Exp0 i 시뮬판: 실차 exp0_braking.py와 같은 함수 (계단 응답 → 순수 + 1차 지연)
        r = step_lag(t, z['final_cmd'][auto], t, v, float(cfg['common']['a_b']))
        out['step_lag'] = r
    out['t_r_eff'] = cfg.get('t_r_eff')
    out['sim_popout_enter_time'] = (cfg.get('timing') or {}).get('popout_enter_time')
    out['v_imp'] = float(meta['config']['tune'].get('v_imp', cfg['common']['v_imp']))
    out['impact_class'] = M.impact_class(out.get('impact_speed'), out['v_imp'])
    out['static_contact'] = bool((z['dist_static'][auto] <= 0.0).any())
    # 실제 d_e (2.10절 정의): #2 앞 끝이 차선 경계에 닿는 순간 에고 앞 끝과 충돌 지점 사이 거리
    out['d_e_actual'] = None
    if meta.get('car2_start') and np.isfinite(z['car2_y']).any():
        c2 = cfg['sim']['car2']
        lane_half = 0.5 * float(cfg['sim']['track']['lane_width'])
        nose_y = z['car2_y'] + (c2['length'] - c2['rear_overhang'])
        hit = np.flatnonzero(nose_y >= -lane_half)
        if len(hit):
            k = hit[0]
            front_x = z['x'][k] + _front_offset(meta) * math.cos(z['yaw'][k])   # 에고 차체 앞 끝 (트랙 x)
            out['d_e_actual'] = float(meta['car2_start'][0] - front_x)           # 충돌 지점 = #2 x (틈 중앙)
            out['car2_enter_after_trigger_s'] = _enter_delay(meta, float(z['t'][k]))
    aeb = z['aeb'][auto].astype(bool)
    out['aeb_triggered'] = bool(aeb.any())
    first = int(np.argmax(aeb)) if aeb.any() else None
    out['aeb_first_s'] = float(s[first]) if first is not None else None
    # 돌출 없는 시나리오에서 정지선 전에 AEB가 걸리면 오작동 (실차 extract_run과 같은 함수)
    out['aeb_malfunction'] = M.aeb_malfunction(meta['scenario'], out['aeb_first_s'], scen['stop_distance'])
    flags = list(z['flag'][auto])
    out['clamped_to_vmin_ratio'] = flags.count('clamped_to_vmin') / len(flags) if flags else None
    frames = [z['boundary_xy'][z['boundary_frame'] == i] for i in range(len(z['frame_t']))]
    std_mean, bins = M.boundary_jitter(frames)
    counts = np.bincount(z['phantoms_frame'].astype(int), minlength=len(z['frame_t'])) \
        if len(z['phantoms_frame']) else np.zeros(len(z['frame_t']), int)
    changes, toggles = M.phantom_flicker(counts)
    out.update(boundary_y_std_mean=std_mean, boundary_bins=bins, boundary_frames=len(frames),
               phantom_count_changes=changes, phantom_on_off_toggles=toggles)
    out['stop_reason_final'] = str(z['stop_reason'][-1]) if len(z['stop_reason']) else ''

    adir = os.path.join(run_dir, 'analysis')
    os.makedirs(adir, exist_ok=True)
    s_max = max([g[0].max() for g in grids.values() if len(g[0])] + [x1])
    grid = np.arange(0.0, s_max, M.DS)
    with open(os.path.join(adir, 'speed_vs_s.csv'), 'w', encoding='utf-8') as f:
        f.write('s,v_raw,v_final,v_meas\n')
        for g in grid:
            vals = [np.interp(g, ss, vs) if len(ss) and g <= ss.max() else math.nan
                    for ss, vs in (grids['cmd'], grids['final'], grids['meas'])]
            f.write(f'{g:.2f},' + ','.join(f'{x:.4f}' for x in vals) + '\n')
    with open(os.path.join(adir, 'metrics.json'), 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return out


if __name__ == '__main__':
    for d in sys.argv[1:]:
        m = compute(d)
        print(f"{m['run']}: dT_cmd={m['dT_cmd']} dT_meas={m['dT_meas']} collision={m['collision']} "
              f"aeb={m['aeb_triggered']} malfunction={m['aeb_malfunction']}")
