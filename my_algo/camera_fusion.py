"""D455 카메라 융합 노드 (ego, 파랑이). 제어 명령은 내지 않는다.

깊이(컬러에 정렬) → 바닥 위 장애물 거리 → /camera/obstacle_scan (LaserScan, /scan과 같은 각도 격자, 라이다 수평 프레임)
컬러 → YOLO 차 상자 → 상자 안 깊이 점 → /camera/lead (JSON: r, a_lo, a_hi, conf, label)
exp_planner(camera_fusion·camera_lead)가 받아 라이다 scan에 합치고 앞차 확인에 쓴다. 계산은 my_algo/aur/camera.py.

시작: 처음 calib_s 동안 바닥 평면으로 카메라 숙임 각도·높이를 재서 터미널에 찍는다 (실패하면 pitch_deg·height).
그 뒤에도 매 프레임 바닥을 다시 맞춰 보고, 보정값에서 3° / 3 cm 안이면 그 프레임 값을 쓴다 (가감속 때 차체 기울기).
YOLO(ultralytics)가 없거나 모델을 못 읽으면 깊이 융합만 한다. 깊이·YOLO 콜백은 따로 돌아 YOLO가 깊이를 막지 않는다.

화면 (show_window / publish_image): 컬러 영상 위에 장애물로 본 깊이 점(주황), YOLO 상자(차 = 초록 + 거리, 그 밖 = 회색),
바닥 보정값·앞차 거리를 그린다. 창은 메인 스레드에서만 띄우고(OpenCV GUI는 스레드를 가린다), 토픽은 /camera/fusion/image.
"""
import json
import math
import statistics
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from my_algo.aur import camera
from my_algo.baseline.vehicle_utils import print_event_line

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import cv2
except ImportError:
    cv2 = None

_DTYPE = {'16UC1': (np.uint16, 1), 'mono16': (np.uint16, 1), '32FC1': (np.float32, 1),
          'bgr8': (np.uint8, 3), 'rgb8': (np.uint8, 3)}


def image_array(msg):
    """sensor_msgs/Image → numpy (cv_bridge 없이). 깊이 (H,W), 컬러 (H,W,3) BGR."""
    dtype, ch = _DTYPE[msg.encoding]
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    a = a.reshape(msg.height, msg.step // np.dtype(dtype).itemsize)[:, :msg.width * ch]
    if ch == 3:
        a = a.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            a = a[..., ::-1]
    return a


class CameraFusion(Node):
    def __init__(self):
        super().__init__('camera_fusion')
        dp = self.declare_parameter
        dp('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        dp('info_topic', '/camera/aligned_depth_to_color/camera_info')
        dp('color_topic', '/camera/color/image_raw')
        # 장착 (실측 2026-11-21): 앞바퀴 축 7 cm 앞 = 라이다(뒤축 앞 0.13 m) 앞 0.27 m, 바닥에서 0.16 m, 15~20° 숙임
        dp('mount_x', 0.27)
        dp('mount_y', 0.0)
        dp('yaw_deg', 0.0)
        dp('pitch_deg', 17.5)       # 바닥 보정 실패 때만 쓰는 값 (+ = 아래를 봄)
        dp('height', 0.16)          # 바닥 → 카메라 [m], 바닥 보정 실패 때만
        dp('calib_s', 1.0)
        dp('floor_refit', True)
        dp('stride', 4)
        dp('min_height', 0.05)      # 바닥 위 이 높이부터 장애물 [m] (깊이 잡음 σ ≈ 0.005·z²)
        dp('max_height', 0.5)
        dp('range_min', 0.3)
        dp('range_max', 3.0)        # D455 깊이 오차는 거리 제곱으로 늘어난다 (3 m에서 약 4.5 cm)
        dp('min_points', 3)
        dp('angle_min', -3.14159)   # /scan(pointcloud_to_laserscan)과 같은 격자
        dp('angle_max', 3.14159)
        dp('angle_increment', 0.00436)
        dp('frame_id', 'livox_level')
        dp('yolo', True)
        dp('yolo_model', 'yolov8n.pt')
        dp('yolo_conf', 0.35)
        dp('yolo_classes', 'car,truck')    # COCO 이름. 직접 학습한 F1TENTH 모델이면 그 클래스 이름 ('*' = 전부)
        dp('yolo_imgsz', 640)
        dp('print_s', 2.0)
        dp('show_window', False)    # OpenCV 창 (차에 화면·원격 데스크톱이 있을 때)
        dp('publish_image', False)  # /camera/fusion/image (rqt_image_view로 다른 PC에서 본다)
        g = self.get_parameter
        self.mount = (g('mount_x').value, g('mount_y').value, math.radians(g('yaw_deg').value))
        self.nominal = camera.Calib(math.radians(g('pitch_deg').value), 0.0, g('height').value)
        self.band = dict(min_height=g('min_height').value, max_height=g('max_height').value,
                         range_min=g('range_min').value)
        self.a_min, self.inc = g('angle_min').value, g('angle_increment').value
        self.n_bins = int(math.ceil((g('angle_max').value - self.a_min) / self.inc))
        self.classes = {c.strip() for c in g('yolo_classes').value.split(',') if c.strip()}
        self.K = None
        self.calib = None
        self.calib_fits = []
        self.t_calib0 = None
        self.rng = np.random.default_rng(0)
        self.lock = threading.Lock()
        self.latest = None          # (body, u, v, calib): YOLO가 상자 안 점을 찾을 최신 깊이 프레임
        self.stat = dict(depth=0, refit=0, bins=0, ms=0.0, yolo=0, car=0, lead=0)
        self.t_print = time.monotonic()
        self.model = self._load_yolo()
        self.show = bool(g('show_window').value)
        self.view = self.show or bool(g('publish_image').value)
        if self.view and cv2 is None:
            self.event('OpenCV(cv2) 없음: 화면 표시 끔')
            self.show = self.view = False
        self.frame = None           # 그린 최신 화면 (메인 스레드 창이 가져간다)
        self.last_lead = None

        img_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        depth_group, color_group = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
        self.create_subscription(CameraInfo, g('info_topic').value, self.info_cb, img_qos, callback_group=depth_group)
        self.create_subscription(Image, g('depth_topic').value, self.depth_cb, img_qos, callback_group=depth_group)
        if self.model is not None or self.view:
            self.create_subscription(Image, g('color_topic').value, self.color_cb, img_qos, callback_group=color_group)
        self.image_pub = self.create_publisher(Image, '/camera/fusion/image', 1) if g('publish_image').value and self.view else None
        self.scan_pub = self.create_publisher(LaserScan, '/camera/obstacle_scan', 5)
        self.lead_pub = self.create_publisher(String, '/camera/lead', 5)
        self.event('시작: 깊이 %s, YOLO %s, 장착 x %.2f y %.2f m, 기본 숙임 %.1f° 높이 %.2f m (바닥으로 다시 잰다)' % (
            g('depth_topic').value, 'ON (%s, %s)' % (g('yolo_model').value, ','.join(sorted(self.classes)))
            if self.model is not None else 'OFF', self.mount[0], self.mount[1], g('pitch_deg').value, g('height').value))

    def event(self, msg):
        print_event_line('[CAM] %s' % msg)

    def _load_yolo(self):
        if not self.get_parameter('yolo').value:
            return None
        if YOLO is None:
            self.event('ultralytics 없음: YOLO 끔 (깊이 융합만)')
            return None
        try:
            return YOLO(self.get_parameter('yolo_model').value)
        except Exception as exc:    # 모델 파일 없음·버전 차이: 깊이만으로 계속 간다
            self.event('YOLO 모델 로드 실패 (%s): YOLO 끔' % exc)
            return None

    def info_cb(self, msg):
        k = msg.k if hasattr(msg, 'k') else msg.K
        self.K = (k[0], k[4], k[2], k[5])

    def depth_cb(self, msg):
        if self.K is None:
            return
        t0 = time.perf_counter()
        depth = image_array(msg)
        opt, u, v = camera.deproject(depth, self.K, int(self.get_parameter('stride').value),
                                     0.001 if depth.dtype == np.uint16 else 1.0)
        body = camera.to_body(opt)
        calib = self._calibrated(body)
        if calib is None:
            return
        ranges, n = camera.obstacle_ranges(body, calib, self.mount, self.a_min, self.inc, self.n_bins,
                                           range_max=self.get_parameter('range_max').value,
                                           min_points=int(self.get_parameter('min_points').value), **self.band)
        with self.lock:
            self.latest = (body, u, v, calib)
        out = LaserScan()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.get_parameter('frame_id').value
        out.angle_min, out.angle_increment = self.a_min, self.inc
        out.angle_max = self.a_min + (self.n_bins - 1) * self.inc
        out.range_min, out.range_max = self.band['range_min'], float(self.get_parameter('range_max').value)
        out.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(out)
        s = self.stat
        s['depth'] += 1
        s['bins'] += int(np.isfinite(ranges).sum())
        s['ms'] += (time.perf_counter() - t0) * 1e3
        self._print()

    def _calibrated(self, body):
        """시작 calib_s 동안 바닥 맞춤의 중앙값 → 보정값. 그 뒤에는 프레임마다 맞춘 값이 보정값 근처면 그 값."""
        fit = camera.fit_floor(body, self.rng)
        now = time.monotonic()
        if self.calib is None:
            self.t_calib0 = now if self.t_calib0 is None else self.t_calib0
            if fit is not None:
                self.calib_fits.append(fit[0])
            if now - self.t_calib0 < self.get_parameter('calib_s').value:
                return None
            if self.calib_fits:
                self.calib = camera.Calib(*(statistics.median(f[i] for f in self.calib_fits) for i in range(3)))
                self.event('바닥 보정: 숙임 %.1f°, roll %+.1f°, 높이 %.3f m (%d프레임 중앙값). 기본값과 5° 넘게 다르면 장착 확인' % (
                    math.degrees(self.calib.pitch), math.degrees(self.calib.roll), self.calib.height, len(self.calib_fits)))
            else:
                self.calib = self.nominal
                self.event('바닥 보정 실패 (바닥이 안 보임?): 기본값 숙임 %.1f° 높이 %.2f m 사용' % (
                    math.degrees(self.nominal.pitch), self.nominal.height))
            return self.calib
        if not self.get_parameter('floor_refit').value:
            return self.calib
        c = camera.refit(self.calib, fit)
        self.stat['refit'] += c is not self.calib
        return c

    def color_cb(self, msg):
        with self.lock:
            latest = self.latest
        if latest is None:
            return
        img = image_array(msg)
        dets, best = [], None
        if self.model is not None:
            try:
                res = self.model.predict(img, conf=float(self.get_parameter('yolo_conf').value),
                                         imgsz=int(self.get_parameter('yolo_imgsz').value), verbose=False)
            except Exception as exc:
                self.event('YOLO 추론 실패: %s' % exc)
                res = []
            self.stat['yolo'] += 1
            body, u, v, calib = latest
            for r in res[:1]:
                names = r.names
                for box in r.boxes:
                    label = names[int(box.cls[0])] if isinstance(names, (dict, list)) else str(int(box.cls[0]))
                    xyxy, conf = box.xyxy[0].tolist(), float(box.conf[0])
                    car = '*' in self.classes or label in self.classes
                    lead = camera.lead_from_box(body, u, v, calib, self.mount, xyxy, **self.band) if car else None
                    dets.append((label, conf, xyxy, car, lead))
                    if car:
                        self.stat['car'] += 1
                    if lead is not None and (best is None or lead['r'] < best['r']):
                        best = dict(lead, conf=conf, label=label)
            if best is not None:
                self.stat['lead'] += 1
                self.lead_pub.publish(String(data=json.dumps(best)))
        if self.view:
            frame = self._annotate(np.ascontiguousarray(img).copy(), latest, dets, best)
            with self.lock:
                self.frame = frame
            if self.image_pub is not None:
                out = Image(header=msg.header, height=frame.shape[0], width=frame.shape[1], encoding='bgr8',
                            is_bigendian=0, step=frame.shape[1] * 3, data=frame.tobytes())
                self.image_pub.publish(out)

    def _annotate(self, img, latest, dets, best):
        """장애물 깊이 점·YOLO 상자·보정값을 그린다 (cv2 글자는 영어만 된다)."""
        body, u, v, calib = latest
        X, Y, H = camera.to_lidar(body, calib, self.mount)
        r = np.hypot(X, Y)
        m = ((H >= self.band['min_height']) & (H <= self.band['max_height']) & (r >= self.band['range_min'])
             & (r <= self.get_parameter('range_max').value))
        h, w = img.shape[:2]
        uu, vv = np.clip(u[m], 0, w - 2), np.clip(v[m], 0, h - 2)
        for du in (0, 1):
            for dv in (0, 1):
                img[vv + dv, uu + du] = (0, 140, 255)
        for label, conf, (x1, y1, x2, y2), car, lead in dets:
            color = (0, 220, 0) if car else (150, 150, 150)
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2 if car else 1)
            text = '%s %.2f' % (label, conf) + ('' if lead is None else ' | %.2f m' % lead['r'])
            cv2.putText(img, text, (int(x1), max(14, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2 if car else 1)
        head = 'pitch %.1f deg  height %.3f m  | obstacle pts %d | YOLO %s | lead %s' % (
            math.degrees(calib.pitch), calib.height, int(m.sum()), 'on' if self.model is not None else 'off',
            '-' if best is None else '%.2f m' % best['r'])
        cv2.rectangle(img, (0, 0), (w, 22), (0, 0, 0), -1)
        cv2.putText(img, head, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def _print(self):
        now = time.monotonic()
        dt = now - self.t_print
        if dt < self.get_parameter('print_s').value:
            return
        s, c = self.stat, self.calib
        n = max(1, s['depth'])
        self.event('깊이 %.0f Hz (처리 %.0f ms), 장애물 bin 평균 %.0f, 바닥 재맞춤 %.0f%% (숙임 %.1f° 높이 %.3f m)%s' % (
            s['depth'] / dt, s['ms'] / n, s['bins'] / n, 100.0 * s['refit'] / n, math.degrees(c.pitch), c.height,
            '' if self.model is None else ' | YOLO %.0f Hz, 차 상자 %d, 앞차 위치 %d' % (s['yolo'] / dt, s['car'], s['lead'])))
        self.stat = dict.fromkeys(s, 0)
        self.stat['ms'] = 0.0
        self.t_print = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraFusion()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        if node.show:
            # OpenCV 창은 메인 스레드에서만 그린다. ROS 콜백은 다른 스레드에서 돈다
            spin = threading.Thread(target=executor.spin, daemon=True)
            spin.start()
            while rclpy.ok() and spin.is_alive():
                with node.lock:
                    frame = node.frame
                if frame is not None:
                    try:
                        cv2.imshow('camera_fusion', frame)
                    except cv2.error as exc:
                        node.event('창을 못 띄움 (화면 없음? DISPLAY 확인): %s. 토픽으로 보려면 publish_image' % exc)
                        node.show = False
                        break
                cv2.waitKey(30)
            spin.join()
        else:
            executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if cv2 is not None and node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
