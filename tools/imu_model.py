"""Pure-Python IMU diagnostics. No serial, GUI, or third-party dependencies."""
from dataclasses import dataclass
from math import asin, atan2, cos, degrees, isfinite, radians, sin, sqrt

# 90 degrees clockwise about +Z, viewed from above; +X points toward the POTI.
BOARD_AXES = 'y,-x,z'


def norm(v):
    return sqrt(sum(x*x for x in v))


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def rotate(q, v):
    return multiply(multiply(q, (0, *v)), (q[0], -q[1], -q[2], -q[3]))[1:]


def advance(q, omega, dt):
    """Body-to-reference quaternion, body rates in rad/s, right-hand positive."""
    speed = norm(omega)
    angle = speed * dt / 2
    scale = sin(angle) / speed if speed else 0
    result = multiply(q, (cos(angle), *(scale*x for x in omega)))
    length = norm(result)
    return tuple(x / length for x in result)


def euler(q):
    w, x, y, z = q
    return tuple(map(degrees, (atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                              asin(max(-1, min(1, 2*(w*y-z*x)))),
                              atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))))


def mounting(spec):
    """Signed sensor axes for board X,Y,Z; require a right-handed rotation."""
    tokens = spec.lower().split(',')
    if len(tokens) != 3 or any(t not in ('x', 'y', 'z', '-x', '-y', '-z') for t in tokens):
        raise ValueError('axes must be three signed axes, e.g. x,y,z or y,x,-z')
    indices = ['xyz'.index(t[-1]) for t in tokens]
    signs = [-1 if t.startswith('-') else 1 for t in tokens]
    inversions = sum(indices[i] > indices[j] for i in range(3) for j in range(i+1, 3))
    if len(set(indices)) != 3 or (-1)**inversions * signs[0]*signs[1]*signs[2] != 1:
        raise ValueError('mounting must be a proper rotation, not a reflection')
    return lambda v: tuple(signs[i]*v[indices[i]] for i in range(3))


@dataclass(frozen=True)
class Sample:
    tick: int
    sequence: int
    reference: int
    accel: tuple
    gyro: tuple


def parse(line):
    fields = line.strip().split(',')
    if len(fields) != 10 or fields[0] != 'IMU':
        return None
    try:
        n = list(map(int, fields[1:]))
    except ValueError:
        return None
    if any(not 0 <= x <= 0xffffffff for x in n[:3]) or any(not -32768 <= x <= 32767 for x in n[3:]):
        return None
    return Sample(*n[:3], tuple(x*0.000061 for x in n[3:6]),
                  tuple(x*0.0175 for x in n[6:9]))


class Attitude:
    def __init__(self, axes='x,y,z', gain=1.5, subtract_bias=True, accel_calibration=None):
        if not isfinite(gain) or gain < 0:
            raise ValueError('gain must be finite and nonnegative')
        self.transform = mounting(axes)
        self.accel_offset = tuple(accel_calibration['offset_g']) if accel_calibration else (0., 0., 0.)
        self.accel_scale = tuple(accel_calibration['scale']) if accel_calibration else (1., 1., 1.)
        self.gain, self.subtract_bias = gain, subtract_bias
        self.q = (1., 0., 0., 0.)
        self.bias = (0., 0., 0.)
        self.accel = self.gyro = (0., 0., 0.)
        self.raw_accel = (0., 0., 0.)
        self.last = None
        self.ready = False
        self.calibrating = False
        self.window = []
        self.elapsed = self.yaw_total = 0.
        self.message = 'Place board level; press Enter to reference'
        self.gaps = 0

    def up_vector(self):
        """Estimated world-up expressed in board axes; unavailable until referenced."""
        return rotate((self.q[0], *(-v for v in self.q[1:])), (0, 0, 1)) if self.ready else None

    def reference(self):
        self.ready = False
        self.calibrating = True
        self.window = []
        self.message = 'Keep level and still for 2 s (gyro bias calibration)'

    def update(self, s):
        self.raw_accel = self.transform(s.accel)
        a = self.transform(tuple((s.accel[i]-self.accel_offset[i])*self.accel_scale[i] for i in range(3)))
        g = self.transform(s.gyro)
        self.accel, self.gyro = a, g
        dt = 0
        if self.last is not None:
            dt = ((s.tick - self.last.tick) & 0xffffffff) / 1000
            expected = (self.last.sequence + 1) & 0xffffffff
            if not 0 < dt <= .1 or s.sequence != expected:
                self.ready = self.calibrating = False
                self.gaps += 1
                self.message = 'Stream gap/restart: press Enter again'
                dt = 0
            if s.reference != self.last.reference:
                self.reference()
        self.last = s
        if self.calibrating:
            # The reference assumes top up, within 8 degrees of level.
            level = .9 <= norm(a) <= 1.1 and a[2] / norm(a) >= cos(radians(8))
            if not level or norm(g) > 3:
                self.window = []
                self.message = 'Calibration waiting: hold LEVEL and STILL (check axes)'
                return
            self.window.append(s)
            span = ((s.tick - self.window[0].tick) & 0xffffffff) / 1000
            if span < 2 or len(self.window) < 100:
                self.message = f'Level/still calibration: {span:.1f}/2.0 s'
                return
            vectors = [self.transform(p.gyro) for p in self.window]
            mean = tuple(sum(v[i] for v in vectors)/len(vectors) for i in range(3))
            deviation = max(sqrt(sum((v[i]-mean[i])**2 for v in vectors)/len(vectors)) for i in range(3))
            if deviation > .3:
                self.window = []
                self.message = 'Calibration restarted: angular motion/noise >0.3 dps'
                return
            self.bias = mean  # Measured once; never auto-learned while navigating.
            self.q = (1., 0., 0., 0.)
            self.elapsed = self.yaw_total = 0.
            self.ready, self.calibrating = True, False
            self.window = []
            self.message = 'Referenced: yaw=0 is your starting direction'
            return
        if not self.ready or not dt:
            return
        if any(abs(x) > 490 for x in g) or any(abs(x) > 1.99 for x in self.raw_accel):
            self.ready = False
            self.message = 'Sensor saturated: press Enter to reference again'
            return
        omega = [radians(g[i] - (self.bias[i] if self.subtract_bias else 0)) for i in range(3)]
        self.message = 'Tracking (no magnetic north reference)'
        if self.gain and .9 <= norm(a) <= 1.1:
            # Measured up cross predicted up; no absolute yaw correction.
            up = rotate((self.q[0], *(-v for v in self.q[1:])), (0, 0, 1))
            measured = tuple(x/norm(a) for x in a)
            error = (measured[1]*up[2]-measured[2]*up[1],
                     measured[2]*up[0]-measured[0]*up[2],
                     measured[0]*up[1]-measured[1]*up[0])
            omega = [omega[i] + self.gain*error[i] for i in range(3)]
        elif self.gain:
            self.message = 'Acceleration outside 0.9–1.1 g: gyro propagation only'
        old_yaw = euler(self.q)[2]
        self.q = advance(self.q, omega, dt)
        self.yaw_total += (euler(self.q)[2] - old_yaw + 180) % 360 - 180
        self.elapsed += dt


class Demo:
    """Synthetic sensor stream: initial rest, then repeating three-axis motion."""
    def __init__(self):
        self.sequence = 0
        self.q = (1., 0., 0., 0.)

    def line(self):
        t = self.sequence/60
        phase = int(max(0, t-4) // 4) % 6
        rates = [(0, 0, 30), (20, 0, 0), (-20, 0, 0),
                 (0, 15, 0), (0, -15, 0), (0, 0, -30)][phase] if t >= 4 else (0, 0, 0)
        self.q = advance(self.q, tuple(map(radians, rates)), 1/60)
        a = rotate((self.q[0], *(-v for v in self.q[1:])), (0, 0, 1))
        # Deliberate +0.2 dps yaw bias for --no-bias drift experiments.
        g = (rates[0], rates[1], rates[2]+.2)
        raw = [round(x/.000061) for x in a] + [round(x/.0175) for x in g]
        result = ','.join(map(str, ('IMU', round(t*1000), self.sequence, 1, *raw)))
        self.sequence += 1
        return result
