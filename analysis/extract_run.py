#!/usr/bin/env python3
"""회차 하나의 bag → 위치별 속도 CSV + 지표 JSON (<run_dir>/analysis/).

    python3 analysis/extract_run.py ~/exp_logs/20261110/S1_P_de1.5_run03
    python3 analysis/extract_run.py --all ~/exp_logs/20261110

위치 s = AUTO 시작부터 odom 주행 거리 (scenario yaml의 x와 같은 기준).
명령 기준 손실은 /exp/v_raw (min(v_ref, v_cap), 최소 속도 정책 전)으로, 실측 기준 손실은 odom 속도로 계산한다.
논문 주장(배치 규칙의 효과)의 핵심 근거는 명령 기준이다.
최소 거리·TTC는 /scan 점 중 차선 안쪽(|y_base| <= obstacle_corridor_half) 점과 차체 footprint의 거리로 낸다.
천장 카메라(analysis/ceiling_camera.py) 결과가 있으면 그쪽이 1순위다 (summarize에서 우선 사용).
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag, stamp  # noqa: E402

from my_algo import metrics as M  # noqa: E402
from my_algo.geometry import load_geometry  # noqa: E402

OBSTACLE_CORRIDOR_HALF = 0.30  # 좌측 벽(약 0.35 m)과 주차 박스(0.45 m)는 제외, 차선에 들어온 #2만 남김


def snapshot(run_dir, name):
    """record 시점에 복사한 설정 파일 (config_snapshot/)을 읽는다."""
    with open(os.path.join(run_dir, 'config_snapshot', name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def series(msgs, fn):
    if not msgs:
        return np.empty(0), np.empty(0)
    return np.array([t for t, _ in msgs]), np.array([fn(m) for _, m in msgs], float)


def lidar_offset(bag):
    """bag의 /tf_static에서 base_link -> livox_frame (x, y, yaw)."""
    for _, msg in bag.get('/tf_static', []):
        for tf in msg.transforms:
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'livox_frame':
                q = tf.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return tf.transform.translation.x, tf.transform.translation.y, yaw
    raise RuntimeError('/tf_static에 base_link->livox_frame 없음 (TF launch를 띄우고 기록했는지 확인)')


def scan_distances(scan_msgs, offset, footprint):
    lx, ly, lyaw = offset
    c, s = math.cos(lyaw), math.sin(lyaw)
    out_t, out_d = [], []
    for t, m in scan_msgs:
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        xb, yb = lx + c * x - s * y, ly + s * x + c * y
        sel = (np.abs(yb) <= OBSTACLE_CORRIDOR_HALF) & (xb > 0.0)
        out_t.append(t)
        out_d.append(M.footprint_distance(np.column_stack([xb[sel], yb[sel]]), footprint))
    return np.array(out_t), np.array(out_d)


def rq4(bag, s_of_t):
    """프레임별 가림 경계의 흔들림과 phantom 깜빡임."""
    frames = bag.get('/exp/boundary_frame', [])
    per_bin = {}
    for _, m in frames:
        if not m.cells:
            continue
        xy = np.array([(p.x, p.y) for p in m.cells])
        bins = np.round(xy[:, 0] / 0.1).astype(int)
        for b in np.unique(bins):
            ys = xy[bins == b, 1]
            per_bin.setdefault(int(b), []).append(float(ys[np.argmin(np.abs(ys))]))
    stds = [np.std(v) for v in per_bin.values() if len(v) >= 5]
    counts = [json.loads(m.data)['n_phantoms'] for _, m in bag.get('/exp/phantom_debug', [])]
    changes = int(np.count_nonzero(np.diff(counts))) if len(counts) > 1 else 0
    toggles = int(np.count_nonzero(np.diff((np.array(counts) > 0).astype(int)))) if len(counts) > 1 else 0
    return {'boundary_y_std_mean': float(np.mean(stds)) if stds else None,
            'boundary_bins': len(stds), 'boundary_frames': len(frames),
            'phantom_count_changes': changes, 'phantom_on_off_toggles': toggles}


def extract(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    common = snapshot(run_dir, 'common.yaml')['/**']['ros__parameters']
    scen = snapshot(run_dir, f'scenario_{meta["scenario"]}.yaml')['/**']['ros__parameters']
    geom = load_geometry(os.path.join(run_dir, 'config_snapshot', 'vehicle_geometry.yaml'))
    v_ref = float(common['v_ref'])
    odom_topic = meta.get('odom_topic', '/vesc/odom')
    bag = read_bag(os.path.join(run_dir, 'bag'))

    autos = [(t, m.data) for t, m in bag.get('/autonomous_mode', [])]
    t_auto = next((t for t, on in autos if on), None)
    if t_auto is None:
        raise RuntimeError('AUTO 시작 기록 없음')
    odom = [(stamp(m), m) for _, m in bag.get(odom_topic, []) if stamp(m) >= t_auto]
    t_o = np.array([t for t, _ in odom])
    x = np.array([m.pose.pose.position.x for _, m in odom])
    y = np.array([m.pose.pose.position.y for _, m in odom])
    v_meas = np.array([m.twist.twist.linear.x for _, m in odom])
    s = M.travel_distance(x, y)

    t_raw, v_raw = series([(t, m) for t, m in bag.get('/exp/v_raw', []) if t >= t_auto], lambda m: m.data)
    t_fin, v_fin = series([(t, m) for t, m in bag.get('/exp/final_cmd', []) if t >= t_auto], lambda m: m.data)
    s_cmd, vr_s = M.series_on_s(t_o, s, t_raw, v_raw) if len(t_raw) else (np.empty(0), np.empty(0))
    s_meas, vm_s = M.series_on_s(t_o, s, t_o, v_meas)
    s_fin, vf_s = M.series_on_s(t_o, s, t_fin, v_fin) if len(t_fin) else (np.empty(0), np.empty(0))

    x0, x1 = float(scen['test_x_start']), float(scen['test_x_end'])
    r0, r1 = float(scen['row_x_start']), float(scen['row_x_end'])
    out = {'run': os.path.basename(run_dir), 'method': meta['method'], 'scenario': meta['scenario'],
           'd_e': meta['d_e'], 'run_id': meta['run_id'], 'discard': meta.get('discard', False),
           'discard_reason': meta.get('discard_reason')}
    for tag, ss, vv in (('cmd', s_cmd, vr_s), ('meas', s_meas, vm_s), ('final', s_fin, vf_s)):
        dt, floored = M.loss_delta_t(ss, vv, v_ref, x0, x1)
        out[f'dT_{tag}'] = dt
        out[f'dT_{tag}_floored'] = floored
        out[f'window_loss_pct_{tag}'] = M.window_loss_pct(ss, vv, v_ref, r0, r1)
        out[f'min_speed_{tag}'] = M.min_speed(ss, vv, x0, x1)
        out[f'decel_length_{tag}'] = M.decel_length(ss, vv, v_ref, x0, x1)
    out['vref_reached'] = M.vref_reached(s_meas, vm_s, float(scen['accel_end_x']), v_ref)
    if not out['vref_reached'] and not out['discard']:
        out['discard'], out['discard_reason'] = True, 'vref_not_reached'

    # jerk: 시험 구간에 있는 시간 동안
    t_in = t_o[(s >= x0) & (s <= x1)]
    t_a, ax = series(bag.get('/imu/linear_accel_base', []), lambda m: m.vector.x)
    out['jerk_rms'] = (M.jerk_rms(t_a, ax, float(t_in[0]), float(t_in[-1]))
                       if len(t_in) and len(t_a) else None)
    out['jerk_cutoff_hz'] = M.JERK_CUTOFF_HZ

    t_d, dist = scan_distances(bag.get('/scan', []), lidar_offset(bag), geom.footprint)
    v_at = np.interp(t_d, t_o, v_meas) if len(t_d) else np.empty(0)
    out.update(M.safety_metrics(t_d, dist, v_at) if len(t_d) else {})
    out['aeb_triggered'] = any(m.data for _, m in bag.get('/aeb/active', []) if _ >= t_auto)
    # S2–S4(돌출 없음)에서 AEB 발동 = 오작동. 효율 통계에서 빼고 방법별로 따로 센다 (summarize.py)
    out['aeb_malfunction'] = bool(out['aeb_triggered'] or meta.get('aeb_malfunction')) \
        and meta['scenario'] != 'S1'
    out['aeb_first_cause_point'] = (meta.get('aeb') or {}).get('first_cause_point')
    flags = [m.data for _, m in bag.get('/exp/min_speed_flag', [])]
    out['clamped_to_vmin_ratio'] = (flags.count('clamped_to_vmin') / len(flags)) if flags else None
    out.update(rq4(bag, None))

    adir = os.path.join(run_dir, 'analysis')
    os.makedirs(adir, exist_ok=True)
    grid = np.arange(0.0, max(s.max() if len(s) else 0, x1), M.DS)
    with open(os.path.join(adir, 'speed_vs_s.csv'), 'w', encoding='utf-8') as f:
        f.write('s,v_raw,v_final,v_meas\n')
        for g in grid:
            vals = [np.interp(g, ss, vv) if len(ss) and g <= ss.max() else math.nan
                    for ss, vv in ((s_cmd, vr_s), (s_fin, vf_s), (s_meas, vm_s))]
            f.write(f'{g:.2f},' + ','.join(f'{v:.4f}' for v in vals) + '\n')
    with open(os.path.join(adir, 'metrics.json'), 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('path')
    ap.add_argument('--all', action='store_true', help='path 아래 모든 회차')
    args = ap.parse_args()
    runs = ([os.path.join(args.path, d) for d in sorted(os.listdir(args.path))
             if os.path.exists(os.path.join(args.path, d, 'meta.json'))] if args.all else [args.path])
    for r in runs:
        try:
            m = extract(r)
            print(f'{m["run"]}: dT_cmd={m["dT_cmd"]} dT_meas={m["dT_meas"]} '
                  f'collision={m.get("collision")} discard={m["discard"]}')
        except Exception as exc:
            print(f'{os.path.basename(r)}: 실패 ({exc})', file=sys.stderr)


if __name__ == '__main__':
    main()
