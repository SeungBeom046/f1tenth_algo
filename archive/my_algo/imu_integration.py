"""ROS-independent experimental strapdown integration (SI units)."""
import math


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def rotate(q, v):
    return multiply(multiply(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))[1:]


class ImuIntegration:
    """Initial frame is the stationary sensor frame; gravity defines its plane."""

    def __init__(self, calibration_samples=200, max_dt=0.1):
        self.required = calibration_samples
        self.max_dt = max_dt
        self.samples = []
        self.ready = False
        self.reset_motion()

    def reset_motion(self):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.distance = 0.0
        self.last_time = None
        self.valid_samples = 0
        self.gaps = 0

    @property
    def speed(self):
        return math.sqrt(sum(v*v for v in self.velocity))

    def calibrate(self, acceleration, gyro):
        # A quiet IMU cannot distinguish rest from constant velocity: caller must
        # ensure the vehicle really is stationary, not merely issuing zero RPM.
        if not all(math.isfinite(x) for x in (*acceleration, *gyro)):
            self.samples.clear()
            return False
        if abs(math.sqrt(sum(x*x for x in acceleration)) - 9.80665) > 1.0 or math.sqrt(sum(x*x for x in gyro)) > 0.1:
            self.samples.clear()
            return False
        self.samples.append((tuple(acceleration), tuple(gyro)))
        if len(self.samples) < self.required:
            return False
        means = [tuple(sum(s[k][i] for s in self.samples)/len(self.samples)
                       for i in range(3)) for k in range(2)]
        if max(abs(s[0][i]-means[0][i]) for s in self.samples for i in range(3)) > 0.2:
            self.samples.clear()
            return False
        self.gravity, self.gyro_bias = means
        norm = math.sqrt(sum(x*x for x in self.gravity))
        self.up = tuple(x/norm for x in self.gravity)
        self.ready = True
        return True

    def update(self, stamp, acceleration, gyro):
        if not self.ready or not all(math.isfinite(x) for x in (stamp, *acceleration, *gyro)):
            return False
        if self.last_time is None:
            self.last_time = stamp
            return False
        dt = stamp - self.last_time
        if dt <= 0:
            self.gaps += 1
            return False
        self.last_time = stamp
        if dt > self.max_dt:
            self.gaps += 1
            return False
        omega = tuple(gyro[i]-self.gyro_bias[i] for i in range(3))
        rate = math.sqrt(sum(x*x for x in omega))
        angle = rate*dt
        dq = (math.cos(angle/2), *(x*math.sin(angle/2)/rate for x in omega)) if rate > 1e-12 else (1., 0., 0., 0.)
        self.q = multiply(self.q, dq)
        norm = math.sqrt(sum(x*x for x in self.q))
        self.q = tuple(x/norm for x in self.q)
        world = rotate(self.q, acceleration)
        linear = tuple(world[i]-self.gravity[i] for i in range(3))
        vertical = sum(linear[i]*self.up[i] for i in range(3))
        planar = tuple(linear[i]-vertical*self.up[i] for i in range(3))
        previous = self.speed
        self.velocity = tuple(self.velocity[i]+planar[i]*dt for i in range(3))
        self.distance += (previous+self.speed)*0.5*dt
        self.valid_samples += 1
        return True
