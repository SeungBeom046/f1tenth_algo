"""대화형 뷰어 오버레이·재생 제어 (mjpython + mujoco.viewer.launch_passive, MuJoCo 3.12).

화면 표시만 바꾼다. 시뮬은 runner.Sim.run의 고정 스텝 루프가 그대로 돌고, 이 모듈은 스텝이 끝난 뒤
상태를 읽어 그리기만 한다 (after_step). 일시정지·배속은 벽시계 대기만 바꾸고 시뮬 dt·시드·노드 시계(시뮬 시간)는 그대로다.
→ sim/tests/test_viewer_determinism.py가 같은 시드의 배치·뷰어 결과를 비트 단위로 비교한다.

쓰는 passive viewer API (3.12에서 확인): key_callback(GLFW 키 코드), Handle.user_scn(MjvScene, maxgeom 100000),
Handle.lock(), Handle.set_texts((font, gridpos, 왼쪽 글, 오른쪽 글)), Handle.sync(), Handle.is_running().
MuJoCo 내장 글꼴은 ASCII만 그리므로 화면 글자는 영어다.

키 (MuJoCo 기본 단축키와 겹치지 않는 것만: 기본은 A–Z 표시 토글, 0–5 그룹, Space·화살표·Backspace·Tab·[ ]·+ -·F1–F5):
    Enter  일시정지 / 재개          .  일시정지 중 라이다 1프레임(0.1 s) 진행      9  배속 1x <-> 0.25x
    6      라이다 점                 7  가림 경계 칸                               8  phantom
    End    글자 패널 (Mac: fn + 오른쪽 화살표)
    Home   재시작: 같은 설정·시드로 처음부터 (Mac: fn + 왼쪽 화살표). 지금 회차를 저장하고 _r1, _r2 ... 폴더로 새 회차
MuJoCo 기본 Reset/Run/Pause(Backspace, Space, 왼쪽 패널 버튼)는 이 시뮬 루프에 영향이 없다.
"""
import time

import mujoco
import numpy as np

KEY_ENTER, KEY_PERIOD, KEY_END, KEY_HOME = 257, 46, 269, 268
KEY_6, KEY_7, KEY_8, KEY_9 = 54, 55, 56, 57
KEYS = [('Enter', 'pause / resume'), ('.', 'step 1 lidar frame (0.1 s) while paused'),
        ('9', 'speed 1x <-> 0.25x'), ('6', 'lidar points (green in band / grey out)'),
        ('7', 'occlusion boundary cells (blue)'), ('8', 'phantoms'), ('End', 'text panel (Mac: fn+Right)'),
        ('Home', 'restart run, same seed (Mac: fn+Left)'),
        ('note', 'MuJoCo Reset/Run/Pause buttons do not work here')]

# 색 (RGBA). 방법별 phantom 색은 그림(analysis/plots.py)과 같은 계열
RGBA_IN_BAND = np.array([0.10, 0.75, 0.30, 0.9], np.float32)    # 높이 대역 안 점 (격자에 쓰임)
RGBA_OUT_BAND = np.array([0.55, 0.55, 0.55, 0.5], np.float32)   # 대역 밖 점 (바닥, 박스 위)
RGBA_BOUNDARY = np.array([0.15, 0.45, 0.95, 0.8], np.float32)   # 가림 경계 칸
RGBA_PHANTOM = {'B2': np.array([0.10, 0.65, 0.45, 0.9], np.float32),
                'P': np.array([0.95, 0.60, 0.05, 0.9], np.float32),
                'B0': np.array([0.20, 0.45, 0.85, 0.9], np.float32),
                'B1': np.array([0.95, 0.40, 0.20, 0.9], np.float32)}
COLOR_NAME = {'B2': 'green', 'P': 'orange', 'B0': 'blue', 'B1': 'red'}
EYE3 = np.eye(3).flatten()


class ViewerUI:
    def __init__(self, sim, max_points=4000, clock=time.monotonic, sleep=time.sleep, start_paused=False):
        self.sim = sim
        self.max_points = int(max_points)
        self.clock, self.sleep = clock, sleep
        self.handle = None
        self.paused = bool(start_paused)   # --start-paused: 창이 뜨면 멈춘 상태, Enter로 시작
        self.restart_requested = False
        self.run_index = 0
        self.speed = 1.0
        self.step_until = None        # 프레임 진행 목표 시뮬 시각
        self.show = {'points': True, 'boundary': True, 'phantoms': True, 'text': True}
        self.keys_pending = []
        c = sim.cfg.common
        self.band = (float(c['band_z_min']), float(c['band_z_max']))
        self.lidar_period = 1.0 / float(sim.cfg.sim['lidar']['hz'])
        self.anchor = None            # (벽시계, 시뮬 시각): 배속 기준점
        self.drawn_points = 0         # 마지막으로 그린 라이다 점 수 (테스트·화면 표시용)
        self.dropped = False          # user_scn 한도로 일부를 못 그렸는지
        self.ended = False

    # ------------------------------------------------------------ 연결
    def attach(self, handle):
        self.handle = handle

    def reset(self, sim):
        """재시작: 새 Sim에 붙인다. 오버레이 토글·배속·카메라(뷰어 쪽 상태)는 유지, 재생 상태만 처음으로."""
        self.sim = sim
        self.paused = False
        self.step_until = None
        self.anchor = None
        self.ended = False
        self.restart_requested = False
        self.run_index += 1

    def on_key(self, key):
        """뷰어 스레드에서 불린다. 상태 변경은 after_step에서 처리하도록 쌓기만 한다."""
        self.keys_pending.append(int(key))

    def _apply_keys(self):
        while self.keys_pending:
            key = self.keys_pending.pop(0)
            if key == KEY_ENTER:
                self.paused = not self.paused
                self.step_until = None
                self.anchor = None
            elif key == KEY_PERIOD and self.paused and self.step_until is None:
                # 다음 라이다 프레임 경계까지 진행 (0.1 s)
                n = int(np.floor(self.sim.t / self.lidar_period + 1e-9)) + 1
                self.step_until = round(n * self.lidar_period, 9)
            elif key == KEY_9:
                self.speed = 0.25 if self.speed == 1.0 else 1.0
                self.anchor = None
            elif key == KEY_6:
                self.show['points'] = not self.show['points']
            elif key == KEY_7:
                self.show['boundary'] = not self.show['boundary']
            elif key == KEY_8:
                self.show['phantoms'] = not self.show['phantoms']
            elif key == KEY_END:
                self.show['text'] = not self.show['text']
            elif key == KEY_HOME:
                self.restart_requested = True

    # ------------------------------------------------------------ 루프 훅
    def after_step(self, sim):
        """시뮬 한 스텝 뒤에 불린다. 시뮬 상태는 읽기만 한다. False = 창 닫힘."""
        h = self.handle
        self._apply_keys()
        if self.restart_requested:
            return False                       # 회차를 여기서 끝내고 viewer_session이 저장 후 새로 시작
        if self.step_until is not None and sim.t + 1e-9 >= self.step_until:
            self.step_until = None            # 한 프레임 진행 끝 → 다시 일시정지 상태
        self.draw()
        h.sync()
        if not h.is_running():
            return False
        # 일시정지: 시뮬을 진행하지 않고 화면만 갱신하며 기다린다
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
            return True                        # 프레임 진행 중에는 기다리지 않는다
        # 배속: 벽시계만 맞춘다 (0.25x = 시뮬 1 s를 벽시계 4 s로)
        now = self.clock()
        if self.anchor is None:
            self.anchor = (now, sim.t)
        target = self.anchor[0] + (sim.t - self.anchor[1]) / self.speed
        if target > now:
            self.sleep(target - now)
        elif now - target > 0.5:               # 계산이 느려 밀리면 기준점을 다시 잡는다 (따라잡기 폭주 방지)
            self.anchor = (now, sim.t)
        return True

    def hold_until_closed(self):
        """회차가 끝난 뒤 창을 닫을 때까지 마지막 장면을 둔다 (시뮬은 이미 끝남, 토글만 동작)."""
        self.ended = True
        h = self.handle
        while h.is_running() and not self.restart_requested:
            self._apply_keys()
            self.draw()
            h.sync()
            self.sleep(0.05)

    # ------------------------------------------------------------ 그리기
    def _geom(self, scn, gtype, size, pos, rgba):
        if scn.ngeom >= scn.maxgeom:
            self.dropped = True
            return False
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], gtype, np.asarray(size, np.float64),
                            np.asarray(pos, np.float64), EYE3, rgba)
        scn.ngeom += 1
        return True

    def points_to_draw(self):
        """이번 프레임 라이다 점(월드) → (표시용으로 솎아낸 점, 대역 안 여부). 알고리즘 입력은 건드리지 않는다."""
        pts = getattr(self.sim, 'last_points_world', None)
        if pts is None or len(pts) == 0:
            return np.empty((0, 3)), np.empty(0, bool)
        pts = np.asarray(pts)
        if len(pts) > self.max_points:        # 고정 간격 솎아내기 (난수 안 씀)
            pts = pts[np.linspace(0, len(pts) - 1, self.max_points).astype(int)]
        z = pts[:, 2]                          # 월드 z = base_link z (바닥 0, 평지)
        return pts, (z >= self.band[0]) & (z <= self.band[1])

    def draw(self):
        """매 프레임 user_scn을 비우고(ngeom = 0) 오버레이를 새로 그린다. 누적하지 않는다 (반투명 도형이 겹치면 어두워짐)."""
        h = self.handle
        with h.lock():
            scn = h.user_scn
            scn.ngeom = 0
            self.dropped = False
            self.fill_scene(scn)
        if self.show['text']:
            h.set_texts(self.texts())
        else:
            h.clear_texts()

    def fill_scene(self, scn):
        """오버레이 도형을 scn 뒤에 덧붙인다 (뷰어 user_scn, 또는 테스트의 오프스크린 렌더 장면)."""
        sim = self.sim
        n = 0
        if self.show['points']:
            pts, inb = self.points_to_draw()
            for p, ok in zip(pts, inb):
                if not self._geom(scn, mujoco.mjtGeom.mjGEOM_BOX, (0.012, 0.012, 0.012), p,
                                  RGBA_IN_BAND if ok else RGBA_OUT_BAND):
                    break
                n += 1
        self.drawn_points = n
        frames = sim.frames
        if self.show['boundary'] and frames['boundary']:
            for x, y in frames['boundary'][-1]:
                if not self._geom(scn, mujoco.mjtGeom.mjGEOM_BOX, (0.025, 0.025, 0.004), (x, y, 0.006),
                                  RGBA_BOUNDARY):
                    break
        if self.show['phantoms'] and frames['phantoms']:
            rgba = RGBA_PHANTOM.get(sim.args.method, RGBA_PHANTOM['P'])
            for x, y in frames['phantoms'][-1]:
                if not self._geom(scn, mujoco.mjtGeom.mjGEOM_CYLINDER, (0.05, 0.15, 0.0), (x, y, 0.15), rgba):
                    break

    def texts(self):
        sim = self.sim
        tap = sim.tap
        num = lambda k: tap.get(k)  # noqa: E731

        def f(v, unit=''):
            if v is None:
                return '-'
            v = float(v)
            return 'inf' if not np.isfinite(v) else f'{v:.2f}{unit}'
        state = 'PAUSED (Enter = start/resume)' if self.paused and self.step_until is None else ('STEP' if self.step_until else 'RUN')
        if self.ended:
            state = 'ENDED (saved; Home = restart)'
        de = '' if sim.args.d_e is None else f' d_e={sim.args.d_e:.1f}'
        method = sim.args.method
        rows = [('method / scenario', f'{method} / {sim.args.scenario}{de} seed={sim.args.seed}'
                 + (f'  (restart #{self.run_index})' if self.run_index else '')),
                ('speed actual', f(sim.ego.v, ' m/s')),
                ('speed command', f(num('/exp/final_cmd'), ' m/s')),
                ('v_cap', f(num('/exp/v_cap'), ' m/s')),
                ('d (nearest phantom)', f(num('/exp/phantom_d'), ' m')),
                ('AEB', 'ACTIVE' if tap.get('/aeb/active') else 'off'),
                ('sim time', f'{sim.t:.2f} s'),
                ('playback', f'{self.speed:g}x {state}'),
                ('phantom color', f'{method}: {COLOR_NAME.get(method, "orange")}' +
                 ('' if method in ('B2', 'P') else ' (no phantoms for this method)')),
                ('lidar points', f'{self.drawn_points} drawn (green=in band '
                                 f'{self.band[0]:.2f}-{self.band[1]:.2f} m, grey=out)'),
                ('boundary cells', 'blue')]
        if self.dropped:
            rows.append(('warning', 'user_scn geom limit reached, some overlay not drawn'))
        keys = [(k, d) for k, d in KEYS]
        left = mujoco.mjtGridPos.mjGRID_TOPLEFT
        bottom = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
        return [(None, left, '\n'.join(a for a, _ in rows), '\n'.join(b for _, b in rows)),
                (None, bottom, '\n'.join(a for a, _ in keys), '\n'.join(b for _, b in keys))]
