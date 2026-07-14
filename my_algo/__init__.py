# QoS 설정 (pointcloud_to_laserscan과 호환)
qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

self.subscription = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, qos)