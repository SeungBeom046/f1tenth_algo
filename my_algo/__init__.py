from std_msgs.msg import Float64, Bool

# 조이스틱 활성 상태 구독
self.joy_active = False
self.joy_active_sub = self.create_subscription(
    Bool, '/joy_active', self.joy_active_callback, 10)

# 자율주행 모드 구독
self.auto_mode = False
self.auto_mode_sub = self.create_subscription(
    Bool, '/autonomous_mode', self.auto_mode_callback, 10)

def joy_active_callback(self, msg):
    self.joy_active = msg.data

def auto_mode_callback(self, msg):
    self.auto_mode = msg.data