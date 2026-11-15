#!/usr/bin/env python3
"""회차별 판단 리포트 report.html (오프라인에서 열림: 데이터·그리기 코드를 파일 안에 넣음, 외부 CDN 없음).

    python sim/report.py <회차 폴더> [...]          # 기존 회차에도 생성
    python sim/report.py --index <sweep 폴더>       # 회차별 리포트 목록 + 판정 요약 표 (index.html)

runner가 회차가 끝나면(배치·뷰어 모두) 자동으로 만든다. 로그·지표는 읽기만 한다 (analysis/sim_reader.py가 만든
analysis/metrics.json, log.npz, meta.json). 판정 기준은 sim/config/report_criteria.yaml 한 곳.
"""
import argparse
import glob
import html
import json
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT]

from my_algo import metrics as M  # noqa: E402
from my_algo.speed_policy import V_MIN  # noqa: E402
from my_algo.track_layout import from_params as layout_from_params  # noqa: E402

CRITERIA = os.path.join(HERE, 'config', 'report_criteria.yaml')
METHOD_COLORS = {'B0': '#2a78d6', 'B1': '#eb6834', 'B2': '#1baf7a', 'P': '#eda100'}   # analysis/plots.py와 같음
PASS, WARN, FAIL, INFO, NA = 'pass', 'warn', 'fail', 'info', 'na'
LABEL = {PASS: '통과', WARN: '주의', FAIL: '실패', INFO: '정보', NA: '해당 없음'}


def _load(run_dir):
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    with open(os.path.join(run_dir, 'analysis', 'metrics.json'), encoding='utf-8') as f:
        m = json.load(f)
    z = dict(np.load(os.path.join(run_dir, 'log.npz')))
    return meta, m, z


def _fin(a):
    """JSON용: inf·nan → None, 소수 4자리."""
    out = []
    for v in np.asarray(a, float):
        out.append(None if not np.isfinite(v) else round(float(v), 4))
    return out


def _intervals(mask, s):
    """불리언 마스크가 참인 s 구간 목록 [(s0, s1)]."""
    out, start = [], None
    for i, on in enumerate(mask):
        if on and start is None:
            start = i
        if (not on or i == len(mask) - 1) and start is not None:
            end = i if on else i - 1
            out.append((round(float(s[start]), 3), round(float(s[end]), 3)))
            start = None
    return out


def _params(meta):
    cfg = meta['config']
    c, tune, scen = cfg['common'], cfg.get('tune') or {}, cfg['scenario_params']
    g = lambda k: float(tune.get(k, c[k]))  # noqa: E731
    t_r_eff = cfg.get('t_r_eff') or (float(c['t_r']) + float(cfg['sim']['vehicle']['tau']))
    return {'v_ref': float(c['v_ref']), 'v_imp': g('v_imp'), 'w_min': g('w_min'),
            'w_max': float(tune.get('w_max', c.get('w_max', math.inf))), 't_r_eff': float(t_r_eff),
            'lane_half': 0.5 * float(cfg['sim']['track']['lane_width']), 'scen': scen,
            'accel_end': float(scen['accel_end_x']), 'x0': float(scen['test_x_start']),
            'x1': float(scen['test_x_end']), 'stop': float(scen['stop_distance'])}


def phantom_sets(z):
    """프레임별 phantom 칸 → (새로 생긴 위치들, 2프레임 이상 유지된 위치들, 칸별 첫 등장 시각)."""
    xy, fr, ft = z['phantoms_xy'], z['phantoms_frame'].astype(int), z['frame_t']
    first, held, prev = {}, set(), set()
    for i in range(len(ft)):
        cur = {(int(round(x / 0.05)), int(round(y / 0.05))) for x, y in xy[fr == i]}
        for k in cur:
            first.setdefault(k, float(ft[i]))
            if k in prev:
                held.add(k)
        prev = cur
    pos = lambda k: (k[0] * 0.05, k[1] * 0.05)  # noqa: E731
    return [pos(k) for k in first], [pos(k) for k in held], {pos(k): t for k, t in first.items()}


def expected_p_positions(meta, P):
    """P의 기대 phantom 위치: 박스 열의 통과 가능한 틈 입구 중앙, 열 양 끝 (w_max 넘는 공간은 양쪽 가장자리)."""
    scen = P['scen']
    if scen.get('right_wall') or int(scen.get('n_boxes', 0)) == 0:
        return []
    boxes = layout_from_params(scen).boxes
    face = -P['lane_half']
    pts = [(boxes[0][0], face), (boxes[-1][1], face)]
    for (a0, a1), (b0, b1) in zip(boxes, boxes[1:]):
        w = b0 - a1
        if w > P['w_max']:
            pts += [(a1, face), (b0, face)]
        elif w >= P['w_min']:
            pts.append((0.5 * (a1 + b0), face))
    return pts


def checks(meta, m, z, crit):
    P = _params(meta)
    auto = z['t'] >= float(meta['config']['sim']['sim']['auto_start_time'])
    t, s, v = z['t'][auto], z['s'][auto], z['v'][auto]
    out = []

    def add(key, title, status, detail, bands=()):
        out.append({'key': key, 'title': title, 'status': status, 'detail': detail, 'bands': list(bands)})

    # 1. 충돌
    if not m.get('collision'):
        add('collision', '충돌', PASS, '없음' + (f" (최소 거리 {m['min_distance']:.3f} m)" if m.get('min_distance') is not None
                                                 else ' (#2 없는 시나리오)'))
    else:
        k = int(np.argmax(z['dist_car2'][auto] <= 0.0))
        cls = m.get('impact_class') or M.impact_class(m.get('impact_speed'), P['v_imp'])
        add('collision', '충돌', WARN if cls == 'le_vimp' else FAIL,
            f"충돌 속도 {m['impact_speed']:.2f} m/s {'≤' if cls == 'le_vimp' else '>'} v_imp {P['v_imp']:.3f} "
            f"({'설계 허용 범위' if cls == 'le_vimp' else 'v_cap 보장 깨짐'}), s = {s[k]:.2f} m",
            [(round(float(s[k]) - 0.05, 3), round(float(s[k]) + 0.05, 3))])
    # 2. 명령이 v_cap을 넘지 않음 (시험 구간)
    test = (s >= P['x0']) & (s <= P['x1'])
    vr, vc = z['v_raw'][auto], z['v_cap'][auto]
    ok = np.isfinite(vr) & np.isfinite(vc)
    over = test & ok & (vr > vc + float(crit['cmd_le_vcap']['eps']))
    add('cmd_le_vcap', '명령(v_raw)이 v_cap을 넘지 않음 (시험 구간)', FAIL if over.any() else PASS,
        f"넘는 스텝 {int(over.sum())}개" + (f", 최대 {float(np.max((vr - vc)[over])):.3f} m/s" if over.any() else ''),
        _intervals(over, s))
    # 3. 실제 속도의 명령 추종 (가정한 지연 안)
    tc = crit['tracking']
    win = P['t_r_eff'] + float(tc['window_extra'])
    vcmd = z['v_cmd'][auto]
    ref = np.full_like(v, np.nan)
    j0 = 0
    for i in range(len(t)):
        while t[j0] < t[i] - win:
            j0 += 1
        seg = vcmd[j0:i + 1]
        seg = seg[np.isfinite(seg)]
        if len(seg):
            ref[i] = seg.max()
    exceed = np.where(np.isfinite(ref), v - ref, -np.inf)
    worst = int(np.argmax(exceed))
    bad = exceed > float(tc['tol'])
    add('tracking', f"실제 속도가 명령을 지연 {P['t_r_eff']:.2f}+{tc['window_extra']} s 안에서 따름",
        FAIL if bad.any() else PASS,
        f"초과량 최대 {exceed[worst]:.3f} m/s at s = {s[worst]:.2f} m (기준 {tc['tol']} m/s, t_r_eff = sim.yaml t_r + τ)",
        _intervals(bad, s))
    # 4. 폐기
    add('discard', '폐기', FAIL if m.get('discard') else PASS,
        f"폐기 ({m.get('discard_reason')})" if m.get('discard') else '아님')
    # 5. AEB 오작동
    if meta['scenario'] == 'S1':
        add('aeb_malfunction', 'AEB 오작동 (S2–S4, 정지선 전)', NA,
            f"S1 (돌출 있음). AEB {'발동 s = %.2f m' % m['aeb_first_s'] if m.get('aeb_first_s') is not None else '없음'}")
    else:
        fs = m.get('aeb_first_s')
        add('aeb_malfunction', 'AEB 오작동 (S2–S4, 정지선 전)', FAIL if m.get('aeb_malfunction') else PASS,
            f"정지선 {P['stop']} m 전 발동 s = {fs:.2f} m" if m.get('aeb_malfunction') else
            (f"정지선 뒤 발동만 (s = {fs:.2f} m)" if fs is not None else '발동 없음'),
            [(round(fs - 0.05, 3), round(fs + 0.05, 3))] if m.get('aeb_malfunction') else [])
    # 6. v_ref 도달
    meas, cmin = m.get('meas_at_accel_end'), m.get('cmd_min_accel')
    if meas is None:
        add('vref', 'v_ref 도달 (가속 구간 끝)', FAIL, '가속 구간 끝에 못 감')
    elif meas >= M.VREF_REACH_RATIO * P['v_ref']:
        add('vref', 'v_ref 도달 (가속 구간 끝)', PASS, f"실측 {meas:.3f} m/s ≥ {M.VREF_REACH_RATIO} v_ref")
    elif cmin is not None and cmin < P['v_ref'] - M.CMD_AT_VREF_EPS:
        acc = (s >= 0.05) & (s <= P['accel_end']) & (z['final_cmd'][auto] < P['v_ref'] - M.CMD_AT_VREF_EPS)
        add('vref', 'v_ref 도달 (가속 구간 끝)', WARN,
            f"미도달 실측 {meas:.3f} m/s, 가속 구간에서 알고리즘이 감속 (명령 최저 {cmin:.3f}) → 폐기 아님",
            _intervals(acc, s))
    else:
        add('vref', 'v_ref 도달 (가속 구간 끝)', FAIL, f"명령 v_ref인데 실측 {meas:.3f} m/s 미도달")
    # 7. 최소 속도 정책 플래그 (정보)
    flag = np.asarray(z['flag'][auto]) == 'clamped_to_vmin'
    iv = _intervals(flag, s)
    add('min_speed_flag', '최소 속도 정책(V_MIN 유지) 발생', INFO,
        f"{int(flag.sum())} 스텝 ({flag.sum() * 0.01:.2f} s), 구간 {len(iv)}개: "
        + ', '.join(f'{a:.2f}–{b:.2f}' for a, b in iv[:6]) + (' …' if len(iv) > 6 else ''), iv)
    # 8. phantom 위치
    pc = crit['phantom']
    new, held, first = phantom_sets(z)
    method = meta['method']
    unexpected = []
    if method in ('B0', 'B1'):
        unexpected = new
        why = 'B0·B1은 phantom이 없어야 함'
    elif method == 'B2':
        lim = P['lane_half'] - float(pc['lane_margin'])
        unexpected = [p for p in new if abs(p[1]) < lim]
        why = f'B2: 보이는 차선 안(|y| < {lim:.2f} m)의 경계 phantom'
    else:
        exp = expected_p_positions(meta, P)
        unexpected = [p for p in new if not any(abs(p[0] - ex) <= float(pc['pos_tol']) and
                                                abs(p[1] - ey) <= float(pc['face_tol']) for ex, ey in exp)]
        why = ('P: 기대 위치(통과 가능한 틈 입구 ' + ', '.join(f'{x:.2f}' for x, _ in exp) +
               f' ± {pc["pos_tol"]} m, 열 쪽 면) 밖') if exp else 'P: 이 배치에서는 기대 위치 없음 (연속 벽·박스 없음)'
    ux = sorted(unexpected)
    add('phantom_pos', 'phantom 위치 (방법별 기대)', FAIL if ux else PASS,
        f"고유 위치 {len(new)}개 중 기대 밖 {len(ux)}개 — {why}"
        + ('. 예: ' + ', '.join(f'({x:.2f}, {y:.2f}) t={first[(x, y)]:.1f}s' for x, y in ux[:6]) + (' …' if len(ux) > 6 else '')
           if ux else ''),
        [(round(x - 0.05, 3), round(x + 0.05, 3)) for x, _ in ux[:40]])
    return out, P, (new, held, ux)


def series_data(meta, m, z, P, phantoms):
    auto = z['t'] >= float(meta['config']['sim']['sim']['auto_start_time'])
    k = np.flatnonzero(auto)[::2]               # 0.02 s 간격
    s, t = z['s'][k], z['t'][k]
    xy = lambda xs, ys: [[round(float(a), 4), b] for a, b in zip(xs, _fin(ys))]  # noqa: E731
    vcap = np.where(np.asarray(z['v_cap'][k], float) > 4.0, np.inf, z['v_cap'][k])
    scen = P['scen']
    lay = layout_from_params(scen)
    bands = [{'x0': 0.0, 'x1': P['accel_end'], 'label': '가속 구간', 'kind': 'zone'},
             {'x0': P['x0'], 'x1': P['x1'], 'label': '시험 구간', 'kind': 'zone2'}]
    if lay.boxes:
        bands.append({'x0': lay.row_x_start, 'x1': lay.row_x_end, 'label': '주차열', 'kind': 'zone3'})
    speed = {'xLabel': 's (AUTO 후 주행 거리 = 트랙 x) [m]', 'yLabel': '속도 [m/s]', 'bands': bands, 'series': [
        {'name': '실제 속도', 'color': '#2a78d6', 'pts': xy(s, z['v'][k])},
        {'name': '명령 속도 (final_cmd)', 'color': '#52514e', 'pts': xy(s, z['final_cmd'][k])},
        {'name': 'v_raw = min(v_ref, v_cap)', 'color': '#eb6834', 'pts': xy(s, z['v_raw'][k])},
        {'name': 'v_cap (4 초과 생략)', 'color': '#8e44c9', 'dash': '6 3', 'pts': xy(s, vcap)},
        {'name': 'v_ref', 'color': '#9a9893', 'dash': '2 3', 'pts': [[0, P['v_ref']], [float(s.max()), P['v_ref']]]},
        {'name': 'V_MIN 0.867', 'color': '#c0392b', 'dash': '2 3', 'pts': [[0, V_MIN], [float(s.max()), V_MIN]]}]}
    d = np.asarray(z['d'][k], float)
    dist = {'xLabel': 's [m]', 'yLabel': 'd 최근접 phantom [m]', 'bands': bands, 'series': [
        {'name': 'd', 'color': '#8e44c9', 'pts': xy(s, np.where(d > 8, np.inf, d))}]}
    rho = {'xLabel': 's [m]', 'yLabel': 'ρ (B1 ROI 미관측 비율)', 'bands': bands, 'series': [
        {'name': 'ρ', 'color': '#eb6834', 'pts': xy(s, z['rho'][k])}]}
    # 시간별 이벤트
    ev = []
    for e in meta.get('events', []):
        if e['topic'] == '/trigger':
            ev.append({'x': e['t'], 'label': '트리거'})
    if meta.get('car2_start') and np.isfinite(z['car2_y']).any():
        c2 = meta['config']['sim']['car2']
        nose = z['car2_y'] + (c2['length'] - c2['rear_overhang'])
        hit = np.flatnonzero(nose >= -P['lane_half'])
        if len(hit):
            ev.append({'x': float(z['t'][hit[0]]), 'label': '#2 차선 진입'})
    aeb = np.asarray(z['aeb'], bool)
    if aeb.any():
        ev.append({'x': float(z['t'][np.argmax(aeb)]), 'label': 'AEB'})
    col = np.flatnonzero(z['dist_car2'] <= 0.0)
    if len(col):
        ev.append({'x': float(z['t'][col[0]]), 'label': '충돌'})
    sr = np.asarray(z['stop_reason'])
    for i in range(1, len(sr)):
        if sr[i] != sr[i - 1] and sr[i]:
            ev.append({'x': float(z['t'][i]), 'label': f'정지 사유: {sr[i]}'})
    timeline = {'xLabel': '시뮬 시간 [s]', 'yLabel': '속도 [m/s]', 'vlines': ev, 'series': [
        {'name': '실제 속도', 'color': '#2a78d6', 'pts': xy(t, z['v'][k])},
        {'name': '명령 속도', 'color': '#52514e', 'pts': xy(t, z['final_cmd'][k])},
        {'name': 'AEB (1 = 작동)', 'color': '#c0392b', 'step': True, 'pts': xy(t, aeb[k].astype(float))}]}
    # 위에서 본 궤적
    new, held, ux = phantoms
    kk = np.arange(0, len(z['t']), 5)
    shapes = [{'x0': b['center'][0] - b['half'][0], 'x1': b['center'][0] + b['half'][0],
               'y0': b['center'][1] - b['half'][1], 'y1': b['center'][1] + b['half'][1], 'name': b['name']}
              for b in meta.get('world_boxes', []) if b['name'] not in ('room_right',)]
    top = {'xLabel': 'x [m]', 'yLabel': 'y [m]', 'equal': True, 'shapes': shapes, 'series': [
        {'name': '에고 경로 (뒤축)', 'color': '#2a78d6', 'pts': xy(z['x'][kk], z['y'][kk])}]}
    if np.isfinite(z['car2_x']).any():
        top['series'].append({'name': '#2 경로', 'color': '#e67e22', 'pts': xy(z['car2_x'][kk], z['car2_y'][kk])})
    top['series'] += [
        {'name': 'phantom 새로 생성 위치', 'color': '#eda100', 'mode': 'points',
         'pts': [[round(x, 3), round(y, 3)] for x, y in new]},
        {'name': 'phantom 유지됨 (2프레임 이상)', 'color': '#8e44c9', 'mode': 'points', 'r': 2,
         'pts': [[round(x, 3), round(y, 3)] for x, y in held]},
        {'name': '기대 밖 phantom', 'color': '#c0392b', 'mode': 'rings', 'pts': [[round(x, 3), round(y, 3)] for x, y in ux]}]
    if len(col):
        top['series'].append({'name': '충돌 지점 (에고 뒤축)', 'color': '#c0392b', 'mode': 'cross',
                              'pts': [[round(float(z['x'][col[0]]), 3), round(float(z['y'][col[0]]), 3)]]})
    return speed, dist, rho, timeline, top


def siblings(run_dir, meta):
    """같은 scenario·d_e·seed·[EXP-TUNE]·조향의 다른 방법 회차 (같은 상위 폴더 안)."""
    out = []
    key = (meta['scenario'], meta['d_e'], meta['seed'], json.dumps(meta['config'].get('tune') or {}, sort_keys=True),
           meta.get('steering'))
    for p in sorted(glob.glob(os.path.join(os.path.dirname(run_dir.rstrip('/')), '*', 'meta.json'))):
        d = os.path.dirname(p)
        if os.path.abspath(d) == os.path.abspath(run_dir):
            continue
        try:
            with open(p, encoding='utf-8') as f:
                mm = json.load(f)
            k2 = (mm['scenario'], mm['d_e'], mm['seed'], json.dumps(mm['config'].get('tune') or {}, sort_keys=True),
                  mm.get('steering'))
            if k2 == key and mm['method'] != meta['method'] and os.path.exists(os.path.join(d, 'analysis', 'metrics.json')):
                out.append(d)
        except (OSError, KeyError, ValueError):
            continue
    best = {}
    for d in out:                       # 방법마다 하나 (재시작 _rN보다 원래 회차 우선)
        mm = json.load(open(os.path.join(d, 'meta.json'), encoding='utf-8'))
        best.setdefault(mm['method'], d)
    return [best[k] for k in sorted(best)]


def compare(run_dir, meta, m, z, speed):
    sib = siblings(run_dir, meta)
    if not sib:
        return None
    rows = [(meta['method'], os.path.basename(run_dir), m)]
    for d in sib:
        mm_meta, mm, zz = _load(d)
        rows.append((mm_meta['method'], os.path.basename(d), mm))
        auto = zz['t'] >= float(mm_meta['config']['sim']['sim']['auto_start_time'])
        k = np.flatnonzero(auto)[::2]
        speed['series'].append({'name': f"{mm_meta['method']} v_raw (같은 조건)", 'color': METHOD_COLORS[mm_meta['method']],
                                'dash': '4 2', 'pts': [[round(float(a), 4), b] for a, b in zip(zz['s'][k], _fin(zz['v_raw'][k]))],
                                'hidden': False})
    return rows


JS = r"""
function el(tag, attrs, parent){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const k in attrs)e.setAttribute(k,attrs[k]);if(parent)parent.appendChild(e);return e;}
function chart(id, spec, highlights){
  const host=document.getElementById(id); const W=host.clientWidth||900, H=spec.equal?360:280, m={l:56,r:16,t:12,b:40};
  const svg=el('svg',{width:W,height:H,role:'img'},null); host.appendChild(svg);
  const vis=spec.series.map(s=>!s.hidden);
  const legend=document.createElement('div'); legend.className='legend'; host.appendChild(legend);
  const tip=document.createElement('div'); tip.className='tip'; host.appendChild(tip);
  function ext(){let xs=[],ys=[];spec.series.forEach((s,i)=>{if(!vis[i])return;s.pts.forEach(p=>{if(p[1]!==null){xs.push(p[0]);ys.push(p[1]);}})});
    (spec.shapes||[]).forEach(b=>{xs.push(b.x0,b.x1);ys.push(b.y0,b.y1)});
    if(!xs.length){xs=[0,1];ys=[0,1];}
    let x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);
    if(!spec.equal){y0=Math.min(0,y0);} if(y1-y0<1e-6)y1=y0+1; if(x1-x0<1e-6)x1=x0+1;
    const py=(y1-y0)*0.06; y0-=spec.equal?py:0; y1+=py; return [x0,x1,y0,y1];}
  function draw(){
    while(svg.firstChild)svg.removeChild(svg.firstChild);
    let [x0,x1,y0,y1]=ext(); const pw=W-m.l-m.r, ph=H-m.t-m.b;
    if(spec.equal){const sx=pw/(x1-x0), sy=ph/(y1-y0); if(sx<sy){const c=(y0+y1)/2,h=ph/sx/2;y0=c-h;y1=c+h;}else{const c=(x0+x1)/2,w=pw/sy/2;x0=c-w;x1=c+w;}}
    const X=v=>m.l+(v-x0)/(x1-x0)*pw, Y=v=>m.t+ph-(v-y0)/(y1-y0)*ph;
    svg._X=X; svg._Y=Y; svg._x0=x0; svg._x1=x1;
    (spec.bands||[]).forEach(b=>{el('rect',{x:X(b.x0),y:m.t,width:Math.max(0,X(b.x1)-X(b.x0)),height:ph,class:'band '+b.kind},svg);
      const t=el('text',{x:X(b.x0)+3,y:m.t+11,class:'bandlabel'},svg);t.textContent=b.label;});
    (highlights||[]).forEach(h=>h.bands.forEach(b=>{ if(h.tick){el('rect',{x:X((b[0]+b[1])/2)-1.5,y:m.t+ph-10,width:3,height:10,class:'tk '+h.status},svg);return;}
      el('rect',{x:X(b[0])-1,y:m.t,width:Math.max(2,X(b[1])-X(b[0])+2),height:ph,class:'hl '+h.status},svg);}));
    for(let i=0;i<=5;i++){const v=y0+(y1-y0)*i/5, yy=Y(v); el('line',{x1:m.l,x2:m.l+pw,y1:yy,y2:yy,class:'grid'},svg);
      const t=el('text',{x:m.l-6,y:yy+4,class:'tick','text-anchor':'end'},svg);t.textContent=v.toFixed(2);}
    for(let i=0;i<=8;i++){const v=x0+(x1-x0)*i/8, xx=X(v); el('line',{x1:xx,x2:xx,y1:m.t+ph,y2:m.t+ph+4,class:'axis'},svg);
      const t=el('text',{x:xx,y:m.t+ph+16,class:'tick','text-anchor':'middle'},svg);t.textContent=v.toFixed(1);}
    el('line',{x1:m.l,x2:m.l+pw,y1:m.t+ph,y2:m.t+ph,class:'axis'},svg);
    const xl=el('text',{x:m.l+pw/2,y:H-6,class:'axlabel','text-anchor':'middle'},svg);xl.textContent=spec.xLabel;
    const yl=el('text',{x:12,y:m.t+ph/2,class:'axlabel','text-anchor':'middle',transform:`rotate(-90 12 ${m.t+ph/2})`},svg);yl.textContent=spec.yLabel;
    (spec.shapes||[]).forEach(b=>{el('rect',{x:X(b.x0),y:Y(b.y1),width:X(b.x1)-X(b.x0),height:Y(b.y0)-Y(b.y1),class:'shape'},svg);});
    (spec.vlines||[]).forEach((v,i)=>{if(v.x<x0||v.x>x1)return; el('line',{x1:X(v.x),x2:X(v.x),y1:m.t,y2:m.t+ph,class:'vline'},svg);
      const t=el('text',{x:X(v.x)+3,y:m.t+12+12*(i%4),class:'vlabel'},svg);t.textContent=v.label;});
    spec.series.forEach((s,i)=>{ if(!vis[i])return;
      if(s.mode){ s.pts.forEach(p=>{const cx=X(p[0]),cy=Y(p[1]);
          if(s.mode==='points')el('circle',{cx,cy,r:s.r||3.5,fill:s.color,class:'pt'},svg);
          else if(s.mode==='rings')el('circle',{cx,cy,r:6,fill:'none',stroke:s.color,'stroke-width':2},svg);
          else {el('path',{d:`M${cx-6},${cy-6}L${cx+6},${cy+6}M${cx-6},${cy+6}L${cx+6},${cy-6}`,stroke:s.color,'stroke-width':3},svg);} }); return; }
      let d='',pen=false,prev=null; s.pts.forEach(p=>{ if(p[1]===null){pen=false;return;}
        if(s.step&&pen&&prev!==null){d+=`L${X(p[0])},${Y(prev)}`;}
        d+=(pen?'L':'M')+X(p[0])+','+Y(p[1]); pen=true; prev=p[1];});
      el('path',{d,fill:'none',stroke:s.color,'stroke-width':2,'stroke-dasharray':s.dash||'','stroke-linejoin':'round'},svg);});
    legend.innerHTML=''; spec.series.forEach((s,i)=>{const b=document.createElement('button');b.className='lg'+(vis[i]?'':' off');
      b.innerHTML=`<span class="sw" style="background:${s.color}"></span>${s.name}`; b.onclick=()=>{vis[i]=!vis[i];draw();}; legend.appendChild(b);});
  }
  draw();
  if(!spec.equal){ svg.addEventListener('mousemove',ev=>{const r=svg.getBoundingClientRect(),px=ev.clientX-r.left;
      const xv=svg._x0+(px-m.l)/(W-m.l-m.r)*(svg._x1-svg._x0); let lines=[`${spec.xLabel.split(' ')[0]} = ${xv.toFixed(2)}`];
      spec.series.forEach((s,i)=>{if(!vis[i]||s.mode)return; let best=null,bd=1e9; s.pts.forEach(p=>{const dd=Math.abs(p[0]-xv); if(dd<bd){bd=dd;best=p;}});
        if(best&&best[1]!==null)lines.push(`<span class="sw" style="background:${s.color}"></span>${s.name}: ${best[1].toFixed(3)}`);});
      tip.innerHTML=lines.join('<br>'); tip.style.display='block'; tip.style.left=Math.min(px+14,W-230)+'px'; tip.style.top='8px';
      let c=svg.querySelector('.cross'); if(!c)c=el('line',{class:'cross'},svg); c.setAttribute('x1',px);c.setAttribute('x2',px);c.setAttribute('y1',m.t);c.setAttribute('y2',H-m.b);});
    svg.addEventListener('mouseleave',()=>{tip.style.display='none'; const c=svg.querySelector('.cross'); if(c)c.remove();}); }
}
"""

CSS = """
:root{--bg:#fcfcfb;--fg:#0b0b0b;--muted:#52514e;--grid:#e4e3df;--card:#ffffff;--pass:#1a7f37;--warn:#b26a00;--fail:#c0392b;--info:#2a78d6;
 --zone:rgba(42,120,214,.06);--zone2:rgba(26,175,122,.08);--zone3:rgba(142,68,201,.07)}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#151514;--fg:#ececea;--muted:#a9a8a3;--grid:#2c2c2a;--card:#1d1d1c;
 --zone:rgba(42,120,214,.12);--zone2:rgba(26,175,122,.12);--zone3:rgba(142,68,201,.14)}}
body{background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Apple SD Gothic Neo",system-ui,sans-serif;margin:0;padding:16px;max-width:1100px}
h1{font-size:20px;margin:0 0 4px} h2{font-size:16px;margin:24px 0 8px} .sub{color:var(--muted)}
.summary{font-size:16px;margin:12px 0} .badge{display:inline-block;padding:1px 8px;border-radius:10px;font-weight:600;color:#fff;font-size:12px}
.badge.pass{background:var(--pass)}.badge.warn{background:var(--warn)}.badge.fail{background:var(--fail)}.badge.info{background:var(--info)}.badge.na{background:#8a8985}
table{border-collapse:collapse;width:100%;background:var(--card)} td,th{border-bottom:1px solid var(--grid);padding:6px 8px;text-align:left;vertical-align:top}
.card{background:var(--card);border:1px solid var(--grid);border-radius:8px;padding:8px;margin:8px 0;position:relative}
.legend{display:flex;flex-wrap:wrap;gap:4px;margin-top:4px}.lg{border:1px solid var(--grid);background:transparent;color:var(--fg);border-radius:6px;padding:2px 8px;cursor:pointer;font-size:12px}
.lg.off{opacity:.35;text-decoration:line-through}.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:middle}
.grid{stroke:var(--grid)}.axis{stroke:var(--muted)}.tick{fill:var(--muted);font-size:11px}.axlabel{fill:var(--muted);font-size:12px}
.band.zone{fill:var(--zone)}.band.zone2{fill:var(--zone2)}.band.zone3{fill:var(--zone3)}.bandlabel{fill:var(--muted);font-size:10px}
.hl.fail{fill:rgba(192,57,43,.22)}.tk.fail{fill:#c0392b}.tk.warn{fill:#b26a00}.hl.warn{fill:rgba(178,106,0,.20)}.vline{stroke:var(--muted);stroke-dasharray:3 3}.vlabel{fill:var(--fg);font-size:11px}
.shape{fill:rgba(128,128,128,.35);stroke:var(--muted)}.cross{stroke:var(--muted)}
.tip{position:absolute;display:none;background:var(--card);border:1px solid var(--grid);border-radius:6px;padding:6px 8px;font-size:12px;pointer-events:none;box-shadow:0 2px 6px rgba(0,0,0,.15)}
code{font-size:12px} a{color:var(--info)}
"""


def render(run_dir, crit=None):
    crit = crit or yaml.safe_load(open(CRITERIA, encoding='utf-8'))
    meta, m, z = _load(run_dir)
    items, P, phantoms = checks(meta, m, z, crit)
    speed, dist, rho, timeline, top = series_data(meta, m, z, P, phantoms)
    rows = compare(run_dir, meta, m, z, speed)
    counts = {k: sum(1 for c in items if c['status'] == k) for k in (PASS, WARN, FAIL)}
    # phantom 위치는 위치가 많아 전체 높이 띠 대신 그래프 아래 짧은 눈금으로 표시
    hl = [{'status': c['status'], 'bands': c['bands'], 'tick': c['key'] == 'phantom_pos'}
          for c in items if c['status'] in (WARN, FAIL) and c['bands']]
    title = f"{meta['scenario']} {meta['method']}" + (f" d_e {meta['d_e']}" if meta['d_e'] is not None else '') + f" seed {meta['seed']}"
    tune = meta['config'].get('tune') or {}
    E = html.escape
    parts = [f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
             f'<title>{E(title)} 회차 리포트</title><style>{CSS}</style></head><body>',
             f'<h1>{E(title)}</h1><div class="sub">{E(os.path.basename(run_dir.rstrip("/")))} · 시뮬 {meta.get("sim_time_s", 0):.1f} s · '
             f'git {E(str(meta.get("git_hash", ""))[:8])}{" (dirty)" if meta.get("git_dirty") else ""} · mujoco {E(str(meta.get("mujoco")))}'
             f'{" · [EXP-TUNE] " + E(json.dumps(tune)) if tune else ""}</div>',
             f'<div class="summary"><span class="badge pass">통과 {counts[PASS]}</span> <span class="badge warn">주의 {counts[WARN]}</span> '
             f'<span class="badge fail">실패 {counts[FAIL]}</span></div>',
             '<h2>판정 체크리스트</h2><table><tr><th>항목</th><th>판정</th><th>근거</th></tr>']
    for c in items:
        parts.append(f'<tr><td>{E(c["title"])}</td><td><span class="badge {c["status"]}">{LABEL[c["status"]]}</span></td>'
                     f'<td>{E(c["detail"])}' + ((' <span class="sub">(속도·d 그래프 아래 빨간 눈금)</span>' if c['key'] == 'phantom_pos'
                                                 else ' <span class="sub">(속도·d 그래프에 띠로 표시)</span>')
                                                if c['status'] in (WARN, FAIL) and c['bands'] else '')
                     + '</td></tr>')
    parts.append('</table><div class="sub">기준: <code>sim/config/report_criteria.yaml</code>, 폐기·AEB 오작동·v_ref 비율은 '
                 '<code>my_algo/metrics.py</code> (실차 분석과 같음). 범례를 누르면 선을 켜고 끈다.</div>')
    key = ['dT_cmd', 'dT_meas', 'min_speed_cmd', 'min_speed_meas', 'decel_length_cmd', 'jerk_rms', 'collision', 'impact_speed',
           'min_distance', 'min_ttc', 'd_e_actual', 'aeb_triggered', 'aeb_first_s', 'tracking_lag', 'discard', 'discard_reason']
    parts.append('<h2>주요 지표 (analysis/metrics.json)</h2><table><tr>' + ''.join(f'<th>{k}</th>' for k in key[:8]) + '</tr><tr>'
                 + ''.join(f'<td>{_fmt(m.get(k))}</td>' for k in key[:8]) + '</tr><tr>' + ''.join(f'<th>{k}</th>' for k in key[8:])
                 + '</tr><tr>' + ''.join(f'<td>{_fmt(m.get(k))}</td>' for k in key[8:]) + '</tr></table>')
    parts.append('<h2>위치별 속도</h2><div class="card" id="c_speed"></div>'
                 '<h2>위치별 d (최근접 phantom 거리)</h2><div class="card" id="c_d"></div>'
                 '<h2>위치별 ρ (B1)</h2><div class="card" id="c_rho"></div>'
                 '<h2>시간별 이벤트</h2><div class="card" id="c_time"></div>'
                 '<h2>위에서 본 궤적</h2><div class="card" id="c_top"></div>')
    if rows:
        parts.append('<h2>같은 조건 비교 (같은 scenario·d_e·seed·[EXP-TUNE])</h2><table><tr><th>방법</th><th>회차</th><th>ΔT cmd</th>'
                     '<th>ΔT meas</th><th>충돌</th><th>충돌 속도</th><th>최소 거리</th></tr>')
        for meth, name, mm in rows:
            parts.append(f'<tr><td>{meth}</td><td><a href="../{E(name)}/report.html">{E(name)}</a></td><td>{_fmt(mm.get("dT_cmd"))}</td>'
                         f'<td>{_fmt(mm.get("dT_meas"))}</td><td>{_fmt(mm.get("collision"))}</td><td>{_fmt(mm.get("impact_speed"))}</td>'
                         f'<td>{_fmt(mm.get("min_distance"))}</td></tr>')
        parts.append('</table><div class="sub">다른 방법의 v_raw를 속도 그래프에 점선으로 겹쳐 그렸다.</div>')
    data = json.dumps({'speed': speed, 'd': dist, 'rho': rho, 'time': timeline, 'top': top, 'hl': hl}, ensure_ascii=False)
    parts.append(f'<script>{JS}\nconst D={data};\nchart("c_speed",D.speed,D.hl);chart("c_d",D.d,D.hl);chart("c_rho",D.rho);'
                 'chart("c_time",D.time);chart("c_top",D.top);</script></body></html>')
    path = os.path.join(run_dir, 'report.html')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(parts))
    summary = {'run': os.path.basename(run_dir.rstrip('/')), 'method': meta['method'], 'scenario': meta['scenario'],
               'd_e': meta['d_e'], 'seed': meta['seed'], 'tune': tune, 'counts': counts,
               'checks': {c['key']: c['status'] for c in items}, 'details': {c['key']: c['detail'] for c in items}}
    with open(os.path.join(run_dir, 'analysis', 'report_summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return path, summary


def _fmt(v):
    if v is None:
        return '–'
    if isinstance(v, bool):
        return '예' if v else '아니오'
    if isinstance(v, float):
        return f'{v:.3f}'
    return html.escape(str(v))


def index(sweep_dir):
    runs = []
    for p in sorted(glob.glob(os.path.join(sweep_dir, '*', 'analysis', 'report_summary.json'))):
        with open(p, encoding='utf-8') as f:
            runs.append(json.load(f))
    keys = ['collision', 'cmd_le_vcap', 'tracking', 'discard', 'aeb_malfunction', 'vref', 'min_speed_flag', 'phantom_pos']
    short = {'collision': '충돌', 'cmd_le_vcap': '명령≤v_cap', 'tracking': '추종', 'discard': '폐기', 'aeb_malfunction': 'AEB 오작동',
             'vref': 'v_ref 도달', 'min_speed_flag': 'V_MIN', 'phantom_pos': 'phantom 위치'}
    E = html.escape
    tot = {k: sum(r['counts'][k] for r in runs) for k in (PASS, WARN, FAIL)}
    parts = [f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
             f'<title>Sweep 리포트 목록</title><style>{CSS}</style></head><body><h1>{E(os.path.basename(sweep_dir.rstrip("/")))}</h1>',
             f'<div class="summary">회차 {len(runs)}개 · <span class="badge pass">통과 {tot[PASS]}</span> <span class="badge warn">주의 {tot[WARN]}</span> '
             f'<span class="badge fail">실패 {tot[FAIL]}</span> (항목 합계)</div>',
             '<h2>항목별 실패 회차 수</h2><table><tr>' + ''.join(f'<th>{short[k]}</th>' for k in keys) + '</tr><tr>'
             + ''.join(f'<td>{sum(1 for r in runs if r["checks"].get(k) == FAIL)} 실패 / {sum(1 for r in runs if r["checks"].get(k) == WARN)} 주의</td>' for k in keys)
             + '</tr></table>', '<h2>회차</h2><table><tr><th>회차</th><th>방법</th><th>시나리오</th><th>d_e</th><th>seed</th><th>[EXP-TUNE]</th>'
             + ''.join(f'<th>{short[k]}</th>' for k in keys) + '</tr>']
    for r in runs:
        parts.append(f'<tr><td><a href="{E(r["run"])}/report.html">{E(r["run"])}</a></td><td>{r["method"]}</td><td>{r["scenario"]}</td>'
                     f'<td>{_fmt(r["d_e"])}</td><td>{r["seed"]}</td><td>{E(json.dumps(r["tune"])) if r["tune"] else ""}</td>'
                     + ''.join(f'<td title="{E(r["details"].get(k, ""))}"><span class="badge {r["checks"].get(k, NA)}">{LABEL[r["checks"].get(k, NA)]}</span></td>'
                               for k in keys) + '</tr>')
    parts.append('</table><div class="sub">칸에 마우스를 올리면 근거가 보인다. 기준: sim/config/report_criteria.yaml</div></body></html>')
    path = os.path.join(sweep_dir, 'index.html')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(parts))
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('run_dirs', nargs='*')
    ap.add_argument('--index', metavar='SWEEP_DIR', help='sweep 폴더의 index.html (회차 리포트가 없으면 먼저 만든다)')
    args = ap.parse_args()
    for d in args.run_dirs:
        path, s = render(d)
        print(f"{path}: 통과 {s['counts'][PASS]} / 주의 {s['counts'][WARN]} / 실패 {s['counts'][FAIL]}")
    if args.index:
        for d in sorted(glob.glob(os.path.join(args.index, '*', 'analysis', 'metrics.json'))):
            run = os.path.dirname(os.path.dirname(d))
            if not os.path.exists(os.path.join(run, 'analysis', 'report_summary.json')):
                render(run)
        print(index(args.index))


if __name__ == '__main__':
    main()
