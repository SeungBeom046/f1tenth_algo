"""Intel RealSense D455 카메라 인식/시각화 노드.

현재 역할:
    카메라가 RGB/depth 정보를 정상적으로 받아오는지 확인하고, YOLO 객체 인식,
    객체 거리 추정, 간단한 차선 후보 검출, AEB 경고 표시를 overlay한다.

중요:
    이 노드는 현재 차량 제어 명령을 발행하지 않는다. 실차 주행은 LiDAR 기반으로
    유지하고, 카메라는 perception 검증용으로만 사용한다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    np = None
    CvBridge = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class CameraReactiveDriveNode(Node):
    """D455 RGB/depth perception 결과를 화면과 ROS image topic으로 출력한다."""

    def __init__(self):
        super().__init__('camera_reactive_drive')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('annotated_topic', '/camera/perception/annotated_image')
        self.declare_parameter('status_topic', '/camera/perception/status')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_confidence', 0.35)
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'D455 Camera Perception')
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('enable_lane_detection', True)

        self.bridge = CvBridge() if CvBridge else None
        self.model = self.load_yolo_model()
        self.depth_image = None
        self.depth_stamp = None
        self.frame_count = 0
        self.last_detections = []
        self.last_lane = {'detected': False, 'left': 0, 'right': 0}
        self.aeb_active = False
        self.aeb_status = 'clear'
        self.last_status_time = 0.0

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.color_sub = self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self.color_callback,
            image_qos,
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            image_qos,
        )
        self.aeb_active_sub = self.create_subscription(
            Bool, '/aeb/active', self.aeb_active_callback, 10)
        self.aeb_status_sub = self.create_subscription(
            String, '/aeb/status', self.aeb_status_callback, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 1)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        if self.bridge is None or cv2 is None or np is None:
            self.get_logger().error('cv_bridge/opencv/numpy가 없어 카메라 overlay를 실행할 수 없다.')
        self.get_logger().info('D455 camera perception overlay ready')

    def load_yolo_model(self):
        """YOLO 모델을 선택적으로 로드한다. 실패해도 카메라 overlay는 계속 동작한다."""
        if not self.get_parameter('enable_yolo').value:
            self.get_logger().warn('YOLO 비활성화: 객체 인식 없이 카메라 화면만 표시한다.')
            return None
        if YOLO is None:
            self.get_logger().warn('ultralytics가 없어 YOLO 객체 인식을 건너뛴다.')
            return None
        model_path = self.get_parameter('yolo_model').value
        try:
            return YOLO(model_path)
        except Exception as exc:
            self.get_logger().warn(f'YOLO 모델 로드 실패: {model_path} ({exc})')
            return None

    def depth_callback(self, msg):
        """D455 aligned depth 이미지를 최신 RGB frame과 같은 좌표계로 저장한다."""
        if self.bridge is None:
            return
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_stamp = msg.header.stamp
        except Exception as exc:
            self.get_logger().warn(f'depth 변환 실패: {exc}')

    def aeb_active_callback(self, msg):
        """AEB 작동 여부를 overlay 경고 아이콘에 반영한다."""
        self.aeb_active = bool(msg.data)

    def aeb_status_callback(self, msg):
        """AEB 작동 이유 또는 상태 문구를 저장한다."""
        self.aeb_status = msg.data

    def color_callback(self, msg):
        """RGB frame마다 객체/차선/AEB 정보를 그려서 publish하고 창에 표시한다."""
        if self.bridge is None or cv2 is None or np is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'color 변환 실패: {exc}')
            return

        self.frame_count += 1
        if self.should_process_frame():
            self.last_detections = self.detect_objects(frame)
            self.last_lane = self.detect_lane(frame)

        annotated = frame.copy()
        self.draw_lane_overlay(annotated, self.last_lane)
        self.draw_detections(annotated, self.last_detections)
        self.draw_header(annotated)
        if self.aeb_active:
            self.draw_aeb_warning(annotated)

        self.publish_annotated_image(annotated, msg)
        self.publish_status()
        self.show_window(annotated)

    def should_process_frame(self):
        """YOLO 연산 부하를 줄이기 위해 N frame마다 한 번만 무거운 처리를 한다."""
        interval = max(1, int(self.get_parameter('process_every_n_frames').value))
        return self.frame_count % interval == 0

    def detect_objects(self, frame):
        """YOLO bbox와 depth 중앙값으로 객체 이름/신뢰도/거리를 계산한다."""
        if self.model is None:
            return []
        confidence = float(self.get_parameter('yolo_confidence').value)
        try:
            results = self.model.predict(frame, conf=confidence, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO 추론 실패: {exc}')
            return []

        detections = []
        if not results:
            return detections
        names = results[0].names
        boxes = getattr(results[0], 'boxes', None)
        if boxes is None:
            return detections

        for box in boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy()
            class_id = int(box.cls[0].detach().cpu().item())
            score = float(box.conf[0].detach().cpu().item())
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            distance = self.estimate_object_distance(x1, y1, x2, y2)
            label = names.get(class_id, str(class_id)) if isinstance(names, dict) else str(class_id)
            decision = self.classify_object_action(label, distance)
            detections.append({
                'label': label,
                'confidence': score,
                'distance_m': distance,
                'bbox': (x1, y1, x2, y2),
                'decision': decision,
            })
        return detections

    def estimate_object_distance(self, x1, y1, x2, y2):
        """bbox 중앙부 depth median을 객체 거리로 사용한다."""
        if self.depth_image is None:
            return None
        height, width = self.depth_image.shape[:2]
        cx1 = int(max(0, min(width - 1, x1 + 0.35 * (x2 - x1))))
        cx2 = int(max(0, min(width, x1 + 0.65 * (x2 - x1))))
        cy1 = int(max(0, min(height - 1, y1 + 0.35 * (y2 - y1))))
        cy2 = int(max(0, min(height, y1 + 0.65 * (y2 - y1))))
        if cx2 <= cx1 or cy2 <= cy1:
            return None

        roi = self.depth_image[cy1:cy2, cx1:cx2].astype('float32')
        valid = roi[np.isfinite(roi)]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None

        median = float(np.median(valid))
        if median > 100.0:
            median *= 0.001
        return median

    def classify_object_action(self, label, distance):
        """객체 종류와 거리를 보고 현재는 표시용 판단만 만든다."""
        soft_objects = {'sports ball', 'frisbee', 'teddy bear'}
        vulnerable = {'person', 'bicycle', 'motorcycle', 'dog', 'cat'}
        vehicles = {'car', 'truck', 'bus', 'train'}

        if label in vulnerable:
            return 'STOP_CANDIDATE'
        if label in vehicles:
            return 'WATCH'
        if label in soft_objects:
            return 'IGNORE_CANDIDATE'
        if distance is not None and distance < 1.0:
            return 'WATCH_CLOSE'
        return 'WATCH'

    def detect_lane(self, frame):
        """하단 ROI에서 흰색/노란색 선분을 찾아 차선 후보 여부만 표시한다."""
        if not self.get_parameter('enable_lane_detection').value:
            return {'detected': False, 'left': 0, 'right': 0}
        height, width = frame.shape[:2]
        roi = frame[int(height * 0.55):height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, (0, 0, 150), (180, 70, 255))
        yellow = cv2.inRange(hsv, (15, 60, 80), (40, 255, 255))
        mask = cv2.bitwise_or(white, yellow)
        edges = cv2.Canny(mask, 60, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 35, minLineLength=35, maxLineGap=25)

        left_lines = []
        right_lines = []
        overlay_lines = []
        if lines is not None:
            for line in lines[:, 0]:
                x1, y1, x2, y2 = [int(v) for v in line]
                dx = x2 - x1
                dy = y2 - y1
                if abs(dx) < 5:
                    continue
                slope = dy / float(dx)
                if abs(slope) < 0.35:
                    continue
                shifted = (x1, y1 + int(height * 0.55), x2, y2 + int(height * 0.55))
                overlay_lines.append(shifted)
                if slope < 0:
                    left_lines.append(shifted)
                else:
                    right_lines.append(shifted)

        return {
            'detected': bool(left_lines or right_lines),
            'left': len(left_lines),
            'right': len(right_lines),
            'lines': overlay_lines[:12],
        }

    def draw_detections(self, frame, detections):
        """객체 bbox와 거리/판단 문구를 화면에 그린다."""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color = self.decision_color(det['decision'])
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            distance = det['distance_m']
            distance_text = 'unknown' if distance is None else f'{distance:.2f} m'
            lines = [
                det['label'],
                f'confidence: {det["confidence"]:.2f}',
                f'distance: {distance_text}',
                f'decision: {det["decision"]}',
            ]
            self.draw_label_box(frame, x1, max(0, y1 - 92), lines, color)

    def draw_lane_overlay(self, frame, lane):
        """검출된 차선 후보 선분과 상태를 표시한다."""
        for x1, y1, x2, y2 in lane.get('lines', []):
            cv2.line(frame, (x1, y1), (x2, y2), (80, 220, 255), 2)
        state = 'LANE: YES' if lane.get('detected') else 'LANE: NO'
        cv2.putText(frame, state, (18, frame.shape[0] - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 220, 255), 2)

    def draw_header(self, frame):
        """화면 상단에 카메라 처리 상태를 표시한다."""
        yolo_state = 'ON' if self.model is not None else 'OFF'
        depth_state = 'ON' if self.depth_image is not None else 'WAIT'
        text = f'D455 PERCEPTION | YOLO:{yolo_state} | DEPTH:{depth_state} | CONTROL:DISABLED'
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 38), (0, 0, 0), -1)
        cv2.putText(frame, text, (12, 27), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2)

    def draw_aeb_warning(self, frame):
        """AEB 작동 시 화면 중앙 상단에 경고 아이콘과 이유를 표시한다."""
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (int(w * 0.30), 48), (int(w * 0.70), 118), (0, 0, 210), -1)
        cv2.putText(frame, '!!! AEB WARNING !!!', (int(w * 0.33), 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(frame, self.aeb_status[:42], (int(w * 0.33), 108),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    def draw_label_box(self, frame, x, y, lines, color):
        """객체 정보 텍스트가 잘 보이도록 반투명 박스를 그린다."""
        line_height = 20
        width = max(180, max(len(line) for line in lines) * 11)
        height = line_height * len(lines) + 12
        x2 = min(frame.shape[1] - 1, x + width)
        y2 = min(frame.shape[0] - 1, y + height)
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x2, y2), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
        for index, line in enumerate(lines):
            cv2.putText(frame, line, (x + 8, y + 22 + index * line_height),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def decision_color(self, decision):
        """표시용 판단 결과별 색상을 반환한다."""
        if decision == 'STOP_CANDIDATE':
            return (0, 0, 255)
        if decision == 'IGNORE_CANDIDATE':
            return (120, 220, 120)
        if decision == 'WATCH_CLOSE':
            return (0, 140, 255)
        return (0, 220, 255)

    def publish_annotated_image(self, frame, source_msg):
        """overlay된 이미지를 ROS topic으로 송출한다."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = source_msg.header
            self.annotated_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'annotated image publish 실패: {exc}')

    def publish_status(self):
        """객체/차선/AEB 상태를 JSON 문자열로 발행한다."""
        now = time.time()
        if now - self.last_status_time < 0.2:
            return
        payload = {
            'objects': [
                {
                    'label': det['label'],
                    'confidence': round(det['confidence'], 3),
                    'distance_m': None if det['distance_m'] is None else round(det['distance_m'], 3),
                    'decision': det['decision'],
                }
                for det in self.last_detections
            ],
            'lane_detected': bool(self.last_lane.get('detected')),
            'lane_left_count': int(self.last_lane.get('left', 0)),
            'lane_right_count': int(self.last_lane.get('right', 0)),
            'aeb_active': self.aeb_active,
            'aeb_status': self.aeb_status,
            'camera_control_enabled': False,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)
        self.last_status_time = now

    def show_window(self, frame):
        """GUI 환경이면 OpenCV 창으로 overlay 화면을 띄운다."""
        if not self.get_parameter('show_window').value:
            return
        try:
            cv2.imshow(self.get_parameter('window_name').value, frame)
            cv2.waitKey(1)
        except Exception:
            self.get_logger().warn('OpenCV 창 표시 실패: show_window를 false로 설정해도 된다.')
            self.set_parameters([
                Parameter(
                    'show_window',
                    Parameter.Type.BOOL,
                    False,
                )
            ])


def main(args=None):
    """D455 카메라 perception overlay 노드를 실행한다."""
    rclpy.init(args=args)
    node = CameraReactiveDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
