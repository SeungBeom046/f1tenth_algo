#!/usr/bin/env python3
"""발표·연구 기록용 자료 만들기: 시뮬 영상(mp4), 주요 장면 캡처(png), 결과 그래프(png).

    pip install -r sim/aur_overtake/media/requirements.txt       # pillow, matplotlib, imageio, imageio-ffmpeg
    python sim/aur_overtake/media/make_media.py --out research_record/2026-11-16_sim [--skip-batch]

판단 로직·차량 모델은 시뮬 그대로다 (sim.Sim). 여기서는 보기만 한다: 오프스크린 렌더 + 화면 글자 + 그래프.
"""
import argparse
import json
import math
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import imageio.v2 as imageio  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from sim import Sim, load_cfg  # noqa: E402
from viewer_ui import draw_overlays  # noqa: E402

W_IMG, H_IMG, FPS = 1920, 1080, 30
FONT_PATH = '/System/Library/Fonts/AppleSDGothicNeo.ttc'
STATE_KO = {'WAIT': '대기', 'APPROACH': '접근', 'PROBE': '탐색 (의도 관찰)', 'PASS': '추월', 'MERGE': '복귀',
            'DONE': '추월 완료', 'FOLLOW': '안전 추종', 'RETREAT': '추월 중단'}
STATE_COLOR = {'WAIT': (150, 150, 150), 'APPROACH': (80, 160, 255), 'PROBE': (255, 190, 40), 'PASS': (60, 200, 110),
               'MERGE': (40, 200, 200), 'DONE': (60, 220, 120), 'FOLLOW': (240, 110, 80), 'RETREAT': (230, 80, 80)}
METHOD_KO = {'a': 'a 고정 규칙', 'b': 'b 소극적', 'c': 'c 공격적', 'd': 'd 제안'}
LEADER_KO = {'Y': 'Y · 비켜 줌', 'NY': 'NY · 안 비킴 + 급정지', 'YR': 'YR · 비켰다가 다시 들어옴'}
# 영상으로 남길 회차: 방법 a~d × 앞차 Y/NY + 시뮬 전용 YR (방법 d)
SCENARIOS = [(m, b) for b in ('Y', 'NY') for m in 'abcd'] + [('d', 'YR')]


def ko_event(line):
    """sim 이벤트 줄 → 발표용 한국어 자막."""
    who, msg = (line.split('] ', 1)[-1].strip().split(None, 1) + [''])[:2]
    if who == 'ego':
        if '→' in msg:
            a, rest = msg.split('→', 1)
            b = rest.strip().split(' ')[0]
            return f'추월차: {STATE_KO.get(a.strip(), a.strip())} → {STATE_KO.get(b, b)}'
        if msg.startswith('belief 확정'):
            yn = msg.split()[2]
            return '추월차: 앞차가 ' + ('비켜 준다고 확정' if yn == 'Y' else '안 비킨다고 확정 → 추월 포기')
        if msg.startswith('AEB'):
            return '추월차: AEB 작동' + (' (정지 상태, 잡음)' if '정지 중' in msg else '')
        if msg.startswith('추월 중단'):
            return '추월차: 추월 중단'
        return '추월차: ' + msg
    if msg.startswith('양보 시작'):
        return '앞차: 오른쪽으로 비켜 줌'
    if msg.startswith('양보 끝'):
        return '앞차: 중앙으로 복귀'
    if msg.startswith('급정지'):
        return '앞차: 급정지!'
    if msg.startswith('AEB'):
        return '앞차: AEB 작동'
    return '앞차: ' + msg


def font(size, bold=False):
    try:
        return ImageFont.truetype(FONT_PATH, size, index=6 if bold else 2)
    except OSError:
        return ImageFont.load_default()


def setup_matplotlib():
    from matplotlib import font_manager
    for name in ('Apple SD Gothic Neo', 'AppleGothic'):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = name
            break
    plt.rcParams.update({'axes.unicode_minus': False, 'figure.dpi': 150, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.grid': True, 'grid.alpha': 0.25, 'font.size': 11,
                         'axes.titleweight': 'bold', 'axes.titlesize': 13})


# ---------------------------------------------------------------- 렌더
class Recorder:
    """시뮬을 돌리며 30 fps로 프레임을 만든다. 장면 이벤트(상태 전이 등)마다 캡처도 남긴다."""

    def __init__(self, sim, title, writer, cam='chase'):
        self.sim, self.title, self.cam_mode, self.writer = sim, title, cam, writer
        self.last = None
        self.r = mujoco.Renderer(sim.m, H_IMG, W_IMG)
        self.r_map = mujoco.Renderer(sim.m, 300, 760)
        self.cam = mujoco.MjvCamera()
        self.cam_map = mujoco.MjvCamera()
        self.look = None
        self.next_t = 0.0
        self.snaps = []            # (태그, 이미지)
        self.trace = []            # (t, ego x, y, leader x, y, state, b)
        self.f_title, self.f_big, self.f_mid, self.f_small = font(40, True), font(30, True), font(26), font(22)
        self.prev_state = None
        self.prev_events = 0
        self.stop_after = None

    def _camera(self):
        s = self.sim
        ex, lx = s.ego.kin.x, s.leader.kin.x
        target = np.array([0.5 * (ex + lx) + 0.2, -0.05, 0.05])
        self.look = target if self.look is None else self.look + 0.08 * (target - self.look)
        c = self.cam
        c.lookat[:] = self.look
        c.distance, c.elevation, c.azimuth = 3.4, -24.0, 205.0 if self.cam_mode == 'front' else 22.0
        m = self.cam_map
        m.lookat[:] = [self.look[0] + 0.5, 0.0, 0.0]
        m.distance, m.elevation, m.azimuth = 4.2, -90.0, 90.0

    def _render(self):
        self._camera()
        show = {'points': True, 'track': True, 'lanes': False}
        self.r.update_scene(self.sim.d, self.cam)
        draw_overlays(self.r.scene, self.sim, show, max_points=900)
        img = Image.fromarray(self.r.render())
        self.r_map.update_scene(self.sim.d, self.cam_map)
        draw_overlays(self.r_map.scene, self.sim, {'points': False, 'track': True, 'lanes': True}, labels=False)
        mini = Image.fromarray(self.r_map.render())
        return self._hud(img, mini)

    def _panel(self, d, box, alpha=170):
        x0, y0, x1, y1 = box
        d.rounded_rectangle(box, radius=18, fill=(15, 18, 24, alpha))
        return x0, y0, x1, y1

    def _hud(self, img, mini):
        s, fsm, ego = self.sim, self.sim.fsm, self.sim.ego
        trk = ego.track
        base = img.convert('RGBA')
        over = Image.new('RGBA', base.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(over)
        # 제목 줄
        ft = self.f_title
        for size in (40, 36, 32, 28):      # 미니맵(x 1118~)과 안 겹치게
            ft = font(size, True)
            if d.textlength(self.title, font=ft) <= 1030:
                break
        self._panel(d, (30, 26, 30 + 48 + int(d.textlength(self.title, font=ft)), 100))
        d.text((54, 63 - size * 0.6), self.title, font=ft, fill=(255, 255, 255, 255))
        # 상태 패널
        x0, y0 = 30, 118
        self._panel(d, (x0, y0, 560, y0 + 380))
        col = STATE_COLOR.get(fsm.state, (200, 200, 200))
        d.text((x0 + 24, y0 + 18), '추월차(파랑) 상태', font=self.f_small, fill=(190, 198, 210, 255))
        d.rounded_rectangle((x0 + 24, y0 + 52, x0 + 506, y0 + 104), radius=12, fill=col + (235,))
        d.text((x0 + 42, y0 + 58), STATE_KO.get(fsm.state, fsm.state) + f'  ({fsm.state})', font=self.f_big,
               fill=(15, 18, 24, 255))
        b = fsm.belief.prob_yield()
        d.text((x0 + 24, y0 + 122), '앞차가 비켜 줄 확률  P(Y)', font=self.f_small, fill=(190, 198, 210, 255))
        bx0, by0, bx1 = x0 + 24, y0 + 156, x0 + 420
        d.rounded_rectangle((bx0, by0, bx1, by0 + 26), radius=8, fill=(60, 66, 78, 255))
        d.rounded_rectangle((bx0, by0, bx0 + max(16, int((bx1 - bx0) * b)), by0 + 26), radius=8,
                            fill=(60, 200, 110, 255) if b >= 0.5 else (240, 110, 80, 255))
        for th in (s.p.eta, 1 - s.p.eta):
            xt = bx0 + int((bx1 - bx0) * th)
            d.line((xt, by0 - 6, xt, by0 + 32), fill=(255, 255, 255, 220), width=2)
        d.text((bx1 + 14, by0 - 4), f'{b:.2f}', font=self.f_mid, fill=(255, 255, 255, 255))
        dec = {'Y': '비켜 준다고 판단', 'NY': '안 비킨다고 판단'}.get(fsm.decided, '판단 중…')
        d.text((x0 + 24, y0 + 194), dec, font=self.f_mid, fill=(255, 225, 120, 255) if fsm.decided else (200, 205, 215, 255))
        rows = [('앞차 거리 (측정 / 실제)', '%s / %.2f m' % ('%.2f' % trk.d if trk is not None and trk.valid else '-',
                                                     s.true_gap())),
                ('추월차 속도', f'{ego.kin.v:.2f} m/s'), ('앞차 속도', f'{s.leader.kin.v:.2f} m/s'),
                ('AEB', '작동' if ego.aeb_active else '대기')]
        for i, (k, v) in enumerate(rows):
            yy = y0 + 240 + 33 * i
            d.text((x0 + 24, yy), k, font=self.f_small, fill=(190, 198, 210, 255))
            d.text((x0 + 330, yy), v, font=self.f_small,
                   fill=(255, 90, 90, 255) if (k == 'AEB' and ego.aeb_active) else (255, 255, 255, 255))
        # 미니맵 (위에서 본 모습)
        mx0, my0 = W_IMG - 30 - 760, 26
        self._panel(d, (mx0 - 12, my0 - 10, W_IMG - 18, my0 + 344))
        d.text((mx0 + 4, my0 + 304), '위에서 본 모습 · 주황 = 인식한 앞차 영역, 빨강 = AEB 검사 영역, 보라 = 추월·양보 경로',
               font=self.f_small, fill=(210, 215, 225, 255))
        # 이벤트 자막 (최근 3줄)
        ev = [ko_event(e) for e in s.events if not e.split('] ', 1)[-1].startswith('ego    - →')][-3:]
        if ev:
            self._panel(d, (30, H_IMG - 40 - 44 * len(ev) - 20, 1180, H_IMG - 30))
            for i, line in enumerate(ev):
                d.text((54, H_IMG - 40 - 44 * (len(ev) - i) - 2), line, font=self.f_mid,
                       fill=(255, 255, 255, 255 if i == len(ev) - 1 else 170))
        d.text((W_IMG - 300, H_IMG - 60), f't = {s.t:5.2f} s', font=self.f_big, fill=(255, 255, 255, 230))
        if s.collision:
            msg = {'cars': '추돌! 앞차 급정지에 멈추지 못함', 'ego_wall': '추월차 벽 충돌', 'leader_wall': '앞차 벽 충돌'}.get(
                s.collision, s.collision)
            fb = font(54, True)
            w = d.textlength(msg, font=fb)
            d.rounded_rectangle(((W_IMG - w) / 2 - 40, 690, (W_IMG + w) / 2 + 40, 790), radius=20, fill=(200, 30, 30, 230))
            d.text(((W_IMG - w) / 2, 706), msg, font=fb, fill=(255, 255, 255, 255))
        out = Image.alpha_composite(base, over)
        out.paste(mini, (mx0, my0))
        return out.convert('RGB')

    def hook(self, sim):
        st = sim.fsm.state
        self.trace.append((sim.t, sim.ego.kin.x, sim.ego.kin.y, sim.leader.kin.x, sim.leader.kin.y, st,
                           sim.fsm.belief.prob_yield(), sim.ego.kin.v, sim.leader.kin.v, sim.true_gap()))
        new_event = len(sim.events) > self.prev_events
        if sim.t + 1e-9 >= self.next_t or new_event:
            frame = self._render()
            if sim.t + 1e-9 >= self.next_t:
                self.writer.append_data(np.asarray(frame))
                self.last = frame
                self.next_t += 1.0 / FPS
            if st != self.prev_state or (new_event and any('급정지' in e or '확정' in e for e in sim.events[self.prev_events:])):
                new = ' '.join(sim.events[self.prev_events:])
                tag = st + ('_leader_brake' if '급정지' in new else '_decided' if '확정' in new else '')
                self.snaps.append(('%05.2fs_%s' % (sim.t, tag), frame))
            self.prev_state, self.prev_events = st, len(sim.events)
        # 끝내기: 급정지 뒤 서서 2 s, 또는 급정지 없이 추종(FOLLOW)으로 3 s (b의 T_max, 추월 중단 뒤 등)
        if st == 'FOLLOW' and sim.ego.kin.v < 0.02 and sim.logic.brake_fired:
            self.stop_after = self.stop_after or sim.t + 2.0
        elif st == 'FOLLOW' and not sim.logic.brake_fired:
            self.stop_after = self.stop_after or sim.t + 3.0
        if self.stop_after is not None and sim.t >= self.stop_after:
            return False
        return True


def record(method, behavior, out, d_y=1.0, seed=0, cam='chase'):
    sim = Sim(method, load_cfg(**{'leader.behavior': behavior, 'leader.d_y': d_y}), seed=seed)
    title = f'방법 {METHOD_KO[method]}  ·  앞차 {LEADER_KO[behavior]}  (d_y = {d_y:.1f} m, 통로 폭 1.0 m)'
    name = f'sim_{method}_{behavior}'
    w = imageio.get_writer(os.path.join(out, 'videos', name + '.mp4'), fps=FPS, codec='libx264', quality=None, ffmpeg_params=['-crf', '23', '-preset', 'slow', '-movflags', '+faststart'],
                           macro_block_size=8, pixelformat='yuv420p')
    rec = Recorder(sim, title, w, cam)
    res = sim.run(hook=rec.hook)
    last = rec._render()                    # 마지막 장면 (충돌로 끝나면 충돌 표시) 1.5 s 정지
    rec.snaps.append(('%05.2fs_END_%s' % (sim.t, sim.collision or sim.fsm.state), last))
    for _ in range(int(1.5 * FPS)):
        w.append_data(np.asarray(last))
    w.close()
    for tag, img in rec.snaps:
        img.save(os.path.join(out, 'screenshots', f'{name}_{tag}.jpg'), quality=93)   # 장면 캡처는 jpg (png의 1/5 크기)
    return res, rec.trace, sim.events


def overview(out):
    """출발 직후 전체 배치를 비스듬히 위에서 본 대표 이미지."""
    sim = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0}))
    sim.run(hook=lambda s: s.t < 0.6)
    r = mujoco.Renderer(sim.m, H_IMG, W_IMG)
    for tag, (dist, el, az, look) in {'overview_oblique': (4.2, -28, 35, 1.2), 'overview_front': (3.0, -16, 200, 1.0),
                                      'overview_top': (3.6, -89.9, 90, 1.0)}.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [sim.ego.kin.x + look, 0, 0.05]
        cam.distance, cam.elevation, cam.azimuth = dist, el, az
        r.update_scene(sim.d, cam)
        draw_overlays(r.scene, sim, {'points': True, 'track': True, 'lanes': tag == 'overview_top'})
        Image.fromarray(r.render()).save(os.path.join(out, 'screenshots', f'{tag}.png'))


# ---------------------------------------------------------------- 그래프
def fig_belief(traces, out):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for (beh, tr), c in zip(traces.items(), ('#2f9e5b', '#e8603c')):
        t = [r[0] for r in tr]
        ax.plot(t, [r[6] for r in tr], color=c, lw=2.4, label=f'앞차 {LEADER_KO[beh]}')
    ax.axhline(0.9, ls='--', color='#555', lw=1)
    ax.axhline(0.1, ls='--', color='#555', lw=1)
    ax.text(0.2, 0.92, 'η = 0.9 → 비켜 준다고 확정', fontsize=9, color='#444')
    ax.text(0.2, 0.05, '1-η = 0.1 → 안 비킨다고 확정', fontsize=9, color='#444')
    ax.set(xlabel='시간 [s]', ylabel='P(앞차가 비켜 줌)', ylim=(-0.02, 1.02), title='belief: 앞차 횡 움직임으로 의도 추정 (방법 d)')
    ax.legend(loc='center right', frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', 'belief_Y_vs_NY.png'))
    plt.close(fig)


def fig_traj(tr, out, name, title):
    fig, ax = plt.subplots(figsize=(11, 4.0))
    x0, x1 = min(r[1] for r in tr) - 0.5, max(max(r[1], r[3]) for r in tr) + 0.8
    ax.axhspan(0.5, 0.56, color='#9aa0a6')
    ax.axhspan(-0.56, -0.5, color='#9aa0a6')
    ax.axhline(0, color='#bbb', lw=0.8)       # 통로 중심 (기준선, 차선 아님)
    states = [r[5] for r in tr]
    for st in dict.fromkeys(states):
        idx = [i for i, s in enumerate(states) if s == st]
        c = np.array(STATE_COLOR.get(st, (150, 150, 150))) / 255
        ax.scatter([tr[i][1] for i in idx], [tr[i][2] for i in idx], s=6, color=c, label=f'추월차: {STATE_KO.get(st, st)}')
    ax.plot([r[3] for r in tr], [r[4] for r in tr], color='#222', lw=2.2, label='앞차')
    ax.set(xlim=(x0, x1), ylim=(-0.6, 0.6), xlabel='진행 방향 x [m]', ylabel='횡위치 y [m] (왼쪽 +)', title=title)
    ax.set_aspect('auto')
    ax.legend(loc='upper center', ncol=7, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, -0.2), markerscale=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', name))
    plt.close(fig)


def fig_ny_gap(tr, events, out, m_stop):
    t = np.array([r[0] for r in tr])
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.plot(t, [r[9] for r in tr], color='#1f6fd1', lw=2.4, label='실제 간격 (추월차 앞범퍼 → 앞차 뒷면)')
    ax2 = ax.twinx()
    ax2.plot(t, [r[7] for r in tr], color='#1f6fd1', lw=1.2, ls='--', alpha=0.7, label='추월차 속도')
    ax2.plot(t, [r[8] for r in tr], color='#222', lw=1.2, ls='--', alpha=0.7, label='앞차 속도')
    ax2.set_ylabel('속도 [m/s]')
    ax2.spines['right'].set_visible(True)
    ax2.grid(False)
    ax.axhline(m_stop, color='#e8603c', lw=1.2, ls=':')
    ax.text(0.1, m_stop + 0.03, f'정지 후 남겨야 할 여유 m = {m_stop} m', color='#e8603c', fontsize=9)
    g_end = tr[-1][9]
    ax.annotate(f'정지 간격 {g_end:.2f} m', (t[-1], g_end), xytext=(t[-1] - 1.6, g_end + 0.25), fontsize=10,
                color='#1f6fd1', arrowprops=dict(arrowstyle='->', color='#1f6fd1'))
    tb = next((float(e.split('s]')[0].strip('[ ')) for e in events if '급정지' in e), None)
    if tb is not None:
        ax.axvline(tb, color='#e8603c', lw=1)
        ax.text(tb + 0.1, ax.get_ylim()[1] * 0.92, '앞차 급정지', color='#e8603c', fontsize=10)
    ax.set(xlabel='시간 [s]', ylabel='간격 [m]', title='앞차가 안 비키고 급정지할 때: 안전 거리로 따라가다 멈춤 (방법 d)')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper right', frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', 'NY_gap_and_speed.png'))
    plt.close(fig)


def _batch_job(job):
    method, beh, seed = job
    cfg = load_cfg(**{'leader.behavior': beh, 'lidar.range_noise_std': 0.02, 'sim.max_time': 30.0})
    r = Sim(method, cfg, seed=seed).run()
    return dict(method=method, behavior=beh, seed=seed, **{k: r[k] for k in (
        'success', 'collision', 'pass_entered', 'T_pass', 'T_id', 'decided', 'N_AEB_moving', 'N_AEB_leader_moving',
        'gap_min', 'stop_gap', 'd_y')})


def fig_methods(rows, out):
    methods = 'abcd'
    labels = [METHOD_KO[m] for m in methods]
    g = lambda m, b: [r for r in rows if r['method'] == m and r['behavior'] == b]  # noqa: E731
    fig, axs = plt.subplots(1, 4, figsize=(15, 4.2))
    cols = ['#9aa0a6', '#7fa7d9', '#f0a35a', '#2f9e5b']
    vals = [100 * np.mean([r['success'] for r in g(m, 'Y')]) for m in methods]
    axs[0].bar(labels, vals, color=cols)
    axs[0].set(title='Y: 추월 성공률', ylabel='%', ylim=(0, 105))
    vals = [np.nanmean([r['T_pass'] for r in g(m, 'Y') if r['T_pass'] is not None] or [np.nan]) for m in methods]
    axs[1].bar(labels, vals, color=cols)
    axs[1].set(title='Y: 추월 완료 시간 (성공 회차)', ylabel='s')
    vals = [np.min([r['stop_gap'] for r in g(m, 'NY') if r['stop_gap'] is not None] or [np.nan]) for m in methods]
    axs[2].bar(labels, vals, color=cols)
    axs[2].axhline(0.15, color='#e8603c', ls=':', lw=1.2)
    axs[2].text(-0.4, 0.16, '필요 여유 m = 0.15 m', color='#e8603c', fontsize=8.5)
    axs[2].set(title='NY: 앞차 급정지 후 최소 간격', ylabel='m')
    for i, m in enumerate(methods):
        ny = g(m, 'NY')
        nc = sum(bool(r['collision']) for r in ny)
        if not any(r['stop_gap'] is not None for r in ny):
            axs[2].text(i, 0.01, '급정지\n없음\n(1.5 m 밖\n유지)', ha='center', fontsize=8, color='#555')
        elif nc:
            axs[2].text(i, 0.03, f'충돌\n{nc}/{len(ny)}', ha='center', fontsize=10, color='#c0392b', fontweight='bold')
    vals = [np.mean([r['N_AEB_moving'] for r in g(m, 'NY')]) for m in methods]
    axs[3].bar(labels, vals, color=cols)
    axs[3].set(title='NY: 주행 중 AEB 개입 (회당)', ylabel='회')
    for ax in axs:
        ax.tick_params(axis='x', labelsize=9)
        for p in ax.patches:
            h = p.get_height()
            if np.isfinite(h):
                ax.annotate(f'{h:.2f}' if h < 1 else f'{h:.1f}' if h < 10 else f'{h:.0f}', (p.get_x() + p.get_width() / 2, h),
                            ha='center', va='bottom', fontsize=9)
    n = len(g('d', 'Y'))
    tries = sum(bool(r['pass_entered']) for r in rows if r['behavior'] == 'NY')
    coll = sum(bool(r['collision']) for r in rows)
    fig.suptitle(f'방법 비교 (시뮬, 통로 폭 1.0 m, 라이다 잡음 0.02 m, 조건마다 {n}회, d_y ~ U[0.6, 1.2] m) · '
                 f'NY 추월 시도 {tries}회, 충돌 {coll}회', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', 'method_comparison.png'))
    plt.close(fig)


def fig_dy(rows, out):
    r = [x for x in rows if x['method'] == 'd' and x['behavior'] == 'Y']
    bins = np.arange(0.6, 1.21, 0.1)
    rate, cnt = [], []
    for lo in bins[:-1]:
        s = [x['success'] for x in r if lo <= x['d_y'] < lo + 0.1 + 1e-9]
        rate.append(100 * np.mean(s) if s else np.nan)
        cnt.append(len(s))
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar([f'{lo:.1f}–{lo + 0.1:.1f}' for lo in bins[:-1]], rate, color='#2f9e5b')
    for i, (v, c) in enumerate(zip(rate, cnt)):
        if np.isfinite(v):
            ax.text(i, v + 1.5, f'{v:.0f}%\n(n={c})', ha='center', fontsize=9)
    ax.set(xlabel='앞차가 비키기 시작하는 거리 d_y [m]', ylabel='추월 성공률 [%]', ylim=(0, 115),
           title='방법 d: 앞차 반응 거리별 추월 성공률')
    ax.text(0.02, 0.97, 'd_y < 0.667 m: 설계상 "안 비킨다"로 판정되는 구간', transform=ax.transAxes, fontsize=9,
            va='top', color='#555')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'figures', 'success_by_dy.png'))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=20, help='방법 비교: 조건마다 회차 수')
    ap.add_argument('--skip-batch', action='store_true')
    ap.add_argument('--skip-video', action='store_true')
    args = ap.parse_args()
    for sub in ('videos', 'screenshots', 'figures', 'data'):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)
    setup_matplotlib()
    if not args.skip_video:
        overview(args.out)
        traces, summary = {}, {}
        for method, beh in SCENARIOS:
            res, tr, ev = record(method, beh, args.out)
            if method == 'd' and beh in ('Y', 'NY'):
                traces[beh] = tr
            summary[f'{method}_{beh}'] = res
            with open(os.path.join(args.out, 'data', f'events_{method}_{beh}.txt'), 'w') as f:
                f.write('\n'.join(ev) + '\n')
            print(method, beh, res['final_state'], res['cause'], 'T_pass', res['T_pass'], 't', res['t_end'], flush=True)
        fig_belief(traces, args.out)
        fig_traj(traces['Y'], args.out, 'trajectory_Y.png', '앞차가 비켜 줄 때: 탐색 → 추월 → 복귀 (위에서 본 궤적, 방법 d)')
        fig_traj(traces['NY'], args.out, 'trajectory_NY.png', '앞차가 안 비킬 때: 탐색 → 안전 추종 → 정지 (방법 d)')
        fig_ny_gap(traces['NY'], open(os.path.join(args.out, 'data', 'events_d_NY.txt')).read().splitlines(),
                   args.out, 0.15)
        with open(os.path.join(args.out, 'data', 'single_runs.json'), 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1, default=str)
    if not args.skip_batch:
        jobs = [(m, b, s) for m in 'abcd' for b in ('Y', 'NY') for s in range(args.n)]
        with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
            rows = pool.map(_batch_job, jobs)
        with open(os.path.join(args.out, 'data', 'method_comparison.json'), 'w') as f:
            json.dump(rows, f, ensure_ascii=False, indent=1, default=str)
        fig_methods(rows, args.out)
        fig_dy(rows, args.out)
        print('batch', len(rows))


if __name__ == '__main__':
    main()
