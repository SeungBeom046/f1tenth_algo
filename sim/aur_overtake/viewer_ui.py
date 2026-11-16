"""대화형 뷰어 오버레이·재생 제어 (mjpython + mujoco.viewer.launch_passive).

키 양식은 예전 시뮬(sim/mujoco 브랜치 sim/viewer_ui.py)과 같다. 화면 표시만 바꾸고 시뮬 진행은 건드리지 않는다.
    Enter  일시정지 / 재개        .  일시정지 중 scan 1프레임(0.1 s) 진행      9  배속 1x <-> 0.25x
    5      라이다가 맞히는 판정용 모양 (벽 판, 차 상자) 보기
    6      라이다 점               7  추적 중인 선행차 표시                       8  차선(±δ)·y_ref 표시
    End    글자 패널 (Mac: fn + →)                 Home  재시작: 같은 설정·시드로 처음부터 (Mac: fn + ←)
MuJoCo 기본 Reset/Run/Pause(Backspace, Space, 왼쪽 패널 버튼)는 이 시뮬 루프에 영향이 없다.
내장 글꼴은 ASCII만 그리므로 화면 글자는 영어다.
"""
import time

import mujoco
import numpy as np

from my_algo.aur import safety
from world import SENSE_GROUP

KEY_ENTER, KEY_PERIOD, KEY_END, KEY_HOME = 257, 46, 269, 268
KEY_5, KEY_6, KEY_7, KEY_8, KEY_9 = 53, 54, 55, 56, 57
KEYS = [('Enter', 'pause / resume'), ('.', 'step 1 scan frame (0.1 s) while paused'),
        ('9', 'speed 1x <-> 0.25x'), ('5', 'lidar collision shapes (what the rays hit)'), ('6', 'lidar points (green)'),
        ('7', 'detected leader region (orange) + ego AEB zone (red)'), ('8', 'lane lines +-delta (purple) and y_ref (yellow)'),
        ('End', 'text panel (Mac: fn+Right)'), ('Home', 'restart run, same seed (Mac: fn+Left)'),
        ('note', 'MuJoCo Reset/Run/Pause buttons do not work here')]
EYE3 = np.eye(3).ravel()
RGBA_POINT = np.array([0.10, 0.75, 0.30, 0.9], np.float32)
RGBA_TRACK_FILL = np.array([1.00, 0.55, 0.10, 0.22], np.float32)
RGBA_TRACK_EDGE = np.array([1.00, 0.60, 0.10, 0.95], np.float32)
RGBA_AEB = np.array([0.95, 0.15, 0.15, 0.18], np.float32)
RGBA_AEB_ON = np.array([1.00, 0.05, 0.05, 0.55], np.float32)
RGBA_LANE = np.array([0.62, 0.20, 0.86, 0.8], np.float32)
RGBA_YREF = np.array([0.95, 0.85, 0.10, 0.9], np.float32)


class ViewerUI:
    def __init__(self, sim, max_points=1500, clock=time.monotonic, sleep=time.sleep, start_paused=False):
        self.sim = sim
        self.max_points = int(max_points)
        self.clock, self.sleep = clock, sleep
        self.handle = None
        self.paused = bool(start_paused)
        self.restart_requested = False
        self.run_index = 0
        self.speed = 1.0
        self.step_until = None
        self.show = {'points': True, 'track': True, 'lanes': True, 'text': True}
        self.keys_pending = []
        self.anchor = None
        self.ended = False

    def attach(self, handle):
        self.handle = handle

    def reset(self, sim):
        """재시작: 새 Sim에 붙인다. 토글·배속·카메라는 유지, 재생 상태만 처음으로."""
        self.sim = sim
        self.paused, self.step_until, self.anchor, self.ended = False, None, None, False
        self.restart_requested = False
        self.run_index += 1

    def on_key(self, key):
        """뷰어 스레드에서 불린다. 상태 변경은 after_step에서 처리하도록 쌓기만 한다."""
        self.keys_pending.append(int(key))

    def _apply_keys(self):
        while self.keys_pending:
            key = self.keys_pending.pop(0)
            if key == KEY_ENTER:
                self.paused, self.step_until, self.anchor = not self.paused, None, None
            elif key == KEY_PERIOD and self.paused and self.step_until is None:
                period = self.sim.frame_dt
                self.step_until = round((int(np.floor(self.sim.t / period + 1e-9)) + 1) * period, 9)
            elif key == KEY_9:
                self.speed, self.anchor = (0.25 if self.speed == 1.0 else 1.0), None
            elif key == KEY_5 and self.handle is not None:
                self.handle.opt.geomgroup[SENSE_GROUP] = 1 - self.handle.opt.geomgroup[SENSE_GROUP]
            elif key == KEY_6:
                self.show['points'] = not self.show['points']
            elif key == KEY_7:
                self.show['track'] = not self.show['track']
            elif key == KEY_8:
                self.show['lanes'] = not self.show['lanes']
            elif key == KEY_END:
                self.show['text'] = not self.show['text']
            elif key == KEY_HOME:
                self.restart_requested = True

    def after_step(self, sim):
        """시뮬 한 스텝 뒤에 불린다. False = 창 닫힘 또는 재시작 요청."""
        h = self.handle
        self._apply_keys()
        if self.restart_requested:
            return False
        if self.step_until is not None and sim.t + 1e-9 >= self.step_until:
            self.step_until = None
        self.draw()
        h.sync()
        if not h.is_running():
            return False
        while self.paused and self.step_until is None:
            self.sleep(0.02)
            self._apply_keys()
            if self.restart_requested:
                return False
            self.draw()
            h.sync()
            if not h.is_running():
                return False
        if self.step_until is not None:
            return True
        now = self.clock()
        if self.anchor is None:
            self.anchor = (now, sim.t)
        target = self.anchor[0] + (sim.t - self.anchor[1]) / self.speed
        if target > now:
            self.sleep(target - now)
        elif now - target > 0.5:
            self.anchor = (now, sim.t)
        return True

    def hold_until_closed(self):
        self.ended = True
        h = self.handle
        while h.is_running() and not self.restart_requested:
            self._apply_keys()
            self.draw()
            h.sync()
            self.sleep(0.05)

    def draw(self):
        h = self.handle
        with h.lock():
            h.user_scn.ngeom = 0
            draw_overlays(h.user_scn, self.sim, self.show, self.max_points)
        if self.show['text']:
            h.set_texts(self.texts())
        else:
            h.clear_texts()

    def texts(self):
        sim, fsm, ego = self.sim, self.sim.fsm, self.sim.ego
        trk = ego.track

        def f(v, unit=''):
            return '-' if v is None or not np.isfinite(v) else f'{float(v):.2f}{unit}'
        play = 'PAUSED (Enter = start/resume)' if self.paused and self.step_until is None else (
            'STEP' if self.step_until else 'RUN')
        if self.ended:
            play = 'ENDED (Home = restart)'
        rows = [('method / leader', f'{sim.method} / {sim.cfg["leader"]["behavior"]} seed={sim.seed} '
                                    f'belief={sim.p.belief_model}' + (f'  (restart #{self.run_index})' if self.run_index else '')),
                ('state', f'{fsm.state}' + (f'  decided={fsm.decided}' if fsm.decided else '')),
                ('b (P yield) / lambda', f'{fsm.belief.prob_yield():.2f} / {fsm.belief.lam:+.2f}'),
                ('d measured / true gap', f'{f(trk.d if trk else None, " m")} / {f(sim.true_gap(), " m")}'),
                ('h / v_lim', f'{f(fsm.h, " m")} / {f(fsm.v_lim, " m/s")}'),
                ('v ego / cmd / leader est', f'{f(ego.kin.v)} / {f(ego.v_cmd)} / {f(trk.v_b if trk else None, " m/s")}'),
                ('ego y est / y_ref', f'{f(ego.corridor.ego_lateral)} / {f(ego.lateral.y_ref, " m")}'),
                ('leader', f'd_y={sim.logic.d_y:.2f} yield={sim.logic.yield_triggered} brake={sim.logic.brake_fired}'),
                ('collision', sim.collision or 'none'),
                ('sim time', f'{sim.t:.2f} s'), ('playback', f'{self.speed:g}x {play}')]
        return [(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, '\n'.join(a for a, _ in rows), '\n'.join(b for _, b in rows)),
                (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, '\n'.join(a for a, _ in KEYS), '\n'.join(b for _, b in KEYS))]


def _geom(scn, gtype, size, pos, rgba, label=''):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, np.float64), np.asarray(pos, np.float64), EYE3, rgba)
    if label:
        g.label = label
    scn.ngeom += 1


def _outline(scn, x0, x1, y0, y1, z, w, rgba):
    """바닥(높이 z)에 직사각형 테두리."""
    xm, ym = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    for pos, size in (((xm, y0, z), (0.5 * (x1 - x0), w, 0.002)), ((xm, y1, z), (0.5 * (x1 - x0), w, 0.002)),
                      ((x0, ym, z), (w, 0.5 * (y1 - y0), 0.002)), ((x1, ym, z), (w, 0.5 * (y1 - y0), 0.002))):
        _geom(scn, mujoco.mjtGeom.mjGEOM_BOX, size, pos, rgba)


def draw_overlays(scn, sim, show, max_points=1500, labels=True):
    """scn(mjvScene)에 오버레이 geom을 더한다. 뷰어와 media.py(오프스크린 녹화)가 같이 쓴다.
    show: points / track / lanes 켜기. 시뮬 상태는 읽기만 한다."""
    ego = sim.ego
    if show.get('points') and len(ego.points_world):
        pts = ego.points_world
        if len(pts) > max_points:
            pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(int)]
        for q in pts:
            _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.01, 0, 0], q, RGBA_POINT)
    trk = ego.track
    if show.get('track'):
        L = sim.cfg['vehicle']['length']
        if trk is not None and trk.valid:
            # ego가 scan으로 인식한 선행차: 뒷면(d) ~ 차 길이, 폭은 클러스터 폭 (최소 차폭)
            x0 = ego.kin.x + sim.front_overhang + trk.d
            hw = 0.5 * max(trk.width or 0.0, sim.p.w_car) + 0.02
            _geom(scn, mujoco.mjtGeom.mjGEOM_BOX, [0.5 * L, hw, 0.15], [x0 + 0.5 * L, trk.y_b, 0.15], RGBA_TRACK_FILL,
                  'DETECTED  d=%.2f m' % trk.d if labels else '')
            _outline(scn, x0, x0 + L, trk.y_b - hw, trk.y_b + hw, 0.003, 0.006, RGBA_TRACK_EDGE)
        # ego AEB 검사 영역: 앞범퍼부터 max(고정 거리, v·TTC), 폭 ±(반폭 + 여유)
        a = sim.cfg['aeb']
        if a['enabled']:
            reach = max(a['bumper_emergency_distance'], ego.kin.v * a['ttc_threshold'])
            hw = 0.15 + a['path_margin']
            c, s_ = np.cos(ego.kin.yaw), np.sin(ego.kin.yaw)
            g0 = scn.ngeom
            _geom(scn, mujoco.mjtGeom.mjGEOM_BOX, [0.5 * reach, hw, 0.002],
                  [ego.kin.x + (sim.front_overhang + 0.5 * reach) * c, ego.kin.y + (sim.front_overhang + 0.5 * reach) * s_, 0.004],
                  RGBA_AEB_ON if ego.aeb_active else RGBA_AEB, 'AEB' if (labels and ego.aeb_active) else '')
            if scn.ngeom > g0:
                scn.geoms[g0].mat = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]], np.float32)
    if show.get('lanes'):
        delta = safety.lane_offset(sim.cfg['track']['W'], sim.p)
        xm = 0.5 * (sim.cfg['track']['x_start'] + sim.cfg['track']['x_end'])
        half = 0.5 * (sim.cfg['track']['x_end'] - sim.cfg['track']['x_start'])
        for y in (delta, -delta):
            _geom(scn, mujoco.mjtGeom.mjGEOM_BOX, [half, 0.003, 0.001], [xm, y, 0.0035], RGBA_LANE)
        _geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0, 0],
              [ego.kin.x + sim.front_overhang + 0.3, ego.lateral.y_ref, 0.03], RGBA_YREF)
