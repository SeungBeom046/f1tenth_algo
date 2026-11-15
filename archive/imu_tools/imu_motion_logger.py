"""Independent IMU-only experimental motion recorder. Never publishes controls."""
import csv
import json
import math
from datetime import datetime
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from my_algo.imu_integration import ImuIntegration


class ImuMotionLogger(Node):
    def __init__(self):
        super().__init__('imu_motion_logger')
        for key, value in [('imu_topic', '/livox/imu'), ('logs_dir', 'logs'),
                           ('calibration_samples', 200), ('max_dt', 0.1),
                           ('imu_timeout', 0.5), ('mode_timeout', 1.0),
                           ('acceleration_scale', 1.0)]:
            self.declare_parameter(key, value)
        self.engine = ImuIntegration(self.get_parameter('calibration_samples').value,
                                     self.get_parameter('max_dt').value)
        if self.engine.required < 2 or self.engine.max_dt <= 0:
            raise ValueError('calibration_samples >= 2 and max_dt > 0 required')
        self.directory = Path(self.get_parameter('logs_dir').value).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.active = False
        self.mode = False
        self.trial = 0
        self.last_mode = None
        self.last_imu = None
        self.file = None
        self.publisher = self.create_publisher(String, '/imu_motion/status', 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/autonomous_mode', self.mode_callback, 10)
        self.create_subscription(Bool, '/imu_motion/recalibrate', self.recalibrate, 10)
        self.create_timer(0.2, self.watchdog)
        self.get_logger().warn('IMU-only experimental recorder: keep vehicle stationary before AUTO until calibration ready. Acceleration must be m/s^2; gyro rad/s.')

    def recalibrate(self, msg):
        if msg.data and not self.active and not self.mode:
            self.engine = ImuIntegration(self.engine.required, self.engine.max_dt)

    def mode_callback(self, msg):
        self.last_mode = time.monotonic()
        self.mode = msg.data
        if msg.data and not self.active:
            self.start()
        elif not msg.data and self.active:
            self.finish('auto_off')

    def start(self):
        self.trial += 1
        self.active = True
        self.started = time.monotonic()
        self.started_ready = self.engine.ready
        # Preserve gyro-propagated attitude between trials; velocity assumes rest
        # at the start of each trial (documented operational prerequisite).
        self.engine.velocity = (0., 0., 0.)
        self.engine.distance = 0.
        self.engine.valid_samples = self.engine.gaps = 0
        self.engine.last_time = None
        self.maximum = 0.
        self.invalid = 0
        self.dropout = False
        self.file = open(self.directory / f'imu_{self.session}_{self.trial}.csv', 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['imu_stamp_s', 'elapsed_s', 'speed_mps', 'distance_m', 'status'])

    def imu_callback(self, msg):
        self.last_imu = time.monotonic()
        a = msg.linear_acceleration
        g = msg.angular_velocity
        scale = self.get_parameter('acceleration_scale').value
        acceleration = tuple(x*scale for x in (a.x, a.y, a.z))
        gyro = (g.x, g.y, g.z)
        available = msg.linear_acceleration_covariance[0] != -1 and msg.angular_velocity_covariance[0] != -1
        if not available or not all(math.isfinite(x) for x in (*acceleration, *gyro)):
            if self.active:
                self.invalid += 1
            return
        if not self.engine.ready:
            if not self.mode and self.engine.calibrate(acceleration, gyro):
                self.get_logger().info('IMU calibration ready; start AUTO from rest.')
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.engine.update(stamp, acceleration, gyro) and self.active:
            self.maximum = max(self.maximum, self.engine.speed)
            self.writer.writerow([stamp, time.monotonic()-self.started,
                                  self.engine.speed, self.engine.distance, 'imu_only_drifting'])

    def watchdog(self):
        now = time.monotonic()
        stale = self.last_imu is None or now-self.last_imu > self.get_parameter('imu_timeout').value
        if self.active:
            self.dropout |= stale
            self.file.flush()
            if self.last_mode is not None and now-self.last_mode > self.get_parameter('mode_timeout').value:
                self.finish('mode_timeout')
        message = String()
        message.data = json.dumps({'calibrated': self.engine.ready, 'active': self.active,
                                   'imu_stale': stale, 'speed_mps': self.engine.speed if self.engine.ready and not stale else None,
                                   'distance_m': self.engine.distance if self.active and self.started_ready else None})
        self.publisher.publish(message)

    def finish(self, reason):
        if not self.active:
            return
        elapsed = time.monotonic()-self.started
        complete = self.started_ready and self.engine.valid_samples > 0 and not (self.dropout or self.invalid or self.engine.gaps)
        fields = ['session', 'trial', 'duration_s', 'distance_m', 'avg_speed_mps', 'max_speed_mps', 'valid_samples', 'gaps', 'invalid_samples', 'status', 'end_reason']
        path = self.directory / 'imu_trials.csv'
        header = not path.exists() or path.stat().st_size == 0
        with path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            if header:
                writer.writerow(fields)
            measured = self.started_ready and self.engine.valid_samples > 0
            writer.writerow([self.session, self.trial, elapsed,
                             self.engine.distance if measured else '',
                             self.engine.distance/elapsed if measured and elapsed else '',
                             self.maximum if measured else '', self.engine.valid_samples,
                             self.engine.gaps, self.invalid,
                             'imu_only_drifting' if complete else 'incomplete', reason])
        self.file.close()
        self.file = None
        self.active = False
        self.get_logger().info(f'IMU trial saved: {path} ({reason})')


def main(args=None):
    rclpy.init(args=args)
    node = ImuMotionLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
