"""QMC5883P field plots and optional magnetic heading."""
from collections import deque
import math
import time
from imu_model import mounting
from magnetic_heading import tilt_heading


class Compass:
    def __init__(self, calibration=None, axes=None):
        self.calibration = calibration
        self.axes = axes
        self.transform = mounting(axes) if axes else None
        self.tick = None
        self.usable = False
        self.points = deque(maxlen=1500)  # Last ~30 seconds at 50 Hz.
        self.last_data = None
        self.samples = self.overflows = 0

    def update(self, line):
        try:
            tag, tick, sequence, x, y, z, status = line.split(',')
            tick, sequence, x, y, z, status = map(int, (tick, sequence, x, y, z, status))
            if tag != 'MAG' or not all(-32768 <= v <= 32767 for v in (x, y, z)) or not 0 <= status <= 255:
                return False
        except ValueError:
            return False
        self.usable = (status & 3) == 1
        if status & 2:
            self.overflows += 1
        elif status & 1:
            v = (x/37.5, y/37.5, z/37.5)  # +/-8 G: 3750 counts/G.
            if self.calibration:
                delta = [v[i]-self.calibration['offset_uT'][i] for i in range(3)]
                v = tuple(sum(row[i]*delta[i] for i in range(3)) for row in self.calibration['matrix'])
            self.tick = tick
            self.points.append(v)
            self.samples += 1
            self.last_data = time.monotonic()
        return True

    def text(self):
        if not self.points:
            return 'Compass: waiting for MAG samples (combined firmware required)'
        x, y, z = self.points[-1]
        stale = time.monotonic()-self.last_data > .5
        return (f'Compass {"STALE" if stale else "CAL" if self.calibration else "RAW"}: sensor XYZ [uT] '
                f'{x:.2f}, {y:.2f}, {z:.2f} |B|={math.hypot(x,y,z):.2f} '
                f'samples={self.samples} overflows={self.overflows}')

    def draw(self, canvas):
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        canvas.create_text(width/2, 15, text=('Corrected' if self.calibration else 'Raw') + ' field [uT] - sensor axes', fill='white')
        bound = max(60, max((abs(v) for p in self.points for v in p), default=0)*1.1)
        radius = min(width/6-25, (height-65)/2)
        for panel, (a, b, label) in enumerate(((0,1,'XY'), (0,2,'XZ'), (1,2,'YZ'))):
            cx, cy = width*(panel+.5)/3, height/2+10
            canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline='#46566a')
            canvas.create_line(cx-radius, cy, cx+radius, cy, fill='#46566a')
            canvas.create_line(cx, cy-radius, cx, cy+radius, fill='#46566a')
            canvas.create_text(cx, height-12, text=f'{label}  +/-{bound:.0f}', fill='white')
            coords = [(cx+p[a]/bound*radius, cy-p[b]/bound*radius) for p in self.points]
            if len(coords) > 1:
                canvas.create_line(*(v for p in coords for v in p), fill='#66bfcc')
            if coords:
                x, y = coords[-1]
                canvas.create_oval(x-3, y-3, x+3, y+3, fill='#ffcc66', outline='')

    def heading(self, accel, imu_tick, imu_age, up=None):
        if self.calibration is None or self.transform is None:
            return None, 'Calibration and compass mounting axes required'
        if not self.usable:
            return None, 'No usable compass sample'
        if self.last_data is None or time.monotonic()-self.last_data > .5 or imu_age > .5:
            return None, 'Stale sensor data'
        if imu_tick is None or min((imu_tick-self.tick) & 0xffffffff,
                                   (self.tick-imu_tick) & 0xffffffff) > 100:
            return None, 'IMU and compass samples not synchronized'
        field = self.transform(self.points[-1])
        return tilt_heading(field, accel, self.calibration['field_uT'], up)

    def draw_heading(self, canvas, accel, imu_tick, imu_age, up=None, fused=None):
        heading, reason = fused if fused is not None else self.heading(accel, imu_tick, imu_age, up)
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        cx, cy = width/2, height/2+2
        r = max(10, min(width/2-24, height/2-45))
        canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline='#6b7e94')
        canvas.create_text(cx, cy-r-12, text='POTI / +X forward', fill='white')
        canvas.create_text(cx-r-12, cy, text='L', fill='white')
        canvas.create_text(cx+r+12, cy, text='R', fill='white')
        title = 'Heading unavailable'
        if heading is not None:
            angle = math.radians(heading)
            end = (cx-r*.8*math.sin(angle), cy-r*.8*math.cos(angle))
            canvas.create_line(cx, cy, *end, arrow='last', width=3, fill='#ff7979')
            canvas.create_text(end[0], end[1]-10, text='N', fill='#ff7979')
            title = f'{"Fused" if fused is not None else "Magnetic"} heading {heading:.1f} deg'
        canvas.create_text(cx, 12, text=title, fill='white' if heading is not None else '#ffcc66')
        canvas.create_text(cx, height-13, text=reason, fill='#b9c8d9', width=max(100,width-12))
