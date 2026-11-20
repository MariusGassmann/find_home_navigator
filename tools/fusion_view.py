"""Display STM32 heading telemetry; angles are clockwise from magnetic north."""
from collections import deque
import time

STATES = ('WAITING', 'CALIBRATING', 'ALIGNING', 'TRACKING', 'COASTING', 'UNRELIABLE')
MISSING = -2147483648


class Fusion:
    def __init__(self):
        self.state = None
        self.tick = None
        self.last_data = None
        self.angles = (None, None, None)
        self.history = deque(maxlen=600)
        self.mag_used = False
        self.sector = 0

    def update(self, line):
        try:
            tag, tick, state, gyro, compass, fused, sector, used = line.strip().split(',')
            tick, gyro, compass, fused, sector, used = map(int, (tick, gyro, compass, fused, sector, used))
            if (tag != 'FUSE' or state not in STATES or not 0 <= tick <= 0xffffffff or
                    not 0 <= sector < 8 or used not in (0, 1) or
                    (state in ('TRACKING', 'COASTING') and (gyro == MISSING or fused == MISSING)) or
                    any(v != MISSING and not 0 <= v < 360000 for v in (gyro, compass, fused))):
                return False
        except ValueError:
            return False
        if (self.tick is not None and (tick-self.tick) & 0xffffffff > 1000) or state in STATES[:3]:
            self.history.clear()
        self.state, self.tick, self.last_data = state, tick, time.monotonic()
        self.sector, self.mag_used = sector, bool(used)
        self.angles = tuple(None if v == MISSING else v/1000 for v in (gyro, compass, fused))
        if state in ('TRACKING', 'COASTING'):
            # Unwrap across north; seed new curves against the fused value.
            anchor = self.history[-1][1][2] if self.history else self.angles[2]
            if anchor is not None:
                values = []
                for i, v in enumerate(self.angles):
                    previous = self.history[-1][1][i] if self.history else None
                    base = anchor if previous is None else previous
                    values.append(None if v is None else base+(v-base+180) % 360-180)
                self.history.append((tick, values))
        return True

    def heading(self):
        if self.last_data is None or time.monotonic()-self.last_data > .5:
            return None, 'No fresh STM32 fusion data'
        if self.state not in ('TRACKING', 'COASTING'):
            return None, self.state
        return self.angles[2], ('Gyro fallback: compass missing/rejected' if self.state == 'COASTING'
                                else 'STM32 fused magnetic north')

    def draw(self, canvas):
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        labels = ['--' if a is None else f'{a:.1f}' for a in self.angles]
        canvas.create_text(12, 13, anchor='w', fill='white',
                           text=f'STM32 {self.state or "waiting"}: gyro {labels[0]} / compass {labels[1]} / fused {labels[2]} deg CW; compass used={self.mag_used}')
        colors = ('#8dabff', '#ffcc66', '#80e69b')
        for i, (name, color) in enumerate(zip(('Gyro', 'Compass', 'Fused'), colors)):
            canvas.create_text(width-210+i*70, 32, text=name, fill=color)
        values = [v for _, row in self.history for v in row if v is not None]
        if not values:
            return
        low, high = min(values)-2, max(values)+2
        canvas.create_text(5, 40, anchor='nw', text=f'{high:.0f}', fill='white')
        canvas.create_text(5, height-18, anchor='nw', text=f'{low:.0f}', fill='white')
        canvas.create_text(width-5, height-6, anchor='se', text='last 60 s', fill='white')
        latest = self.history[-1][0]
        for axis, color in enumerate(colors):
            points = []
            for tick, row in self.history:
                if row[axis] is None:
                    if len(points) >= 4:
                        canvas.create_line(*points, fill=color, width=2)
                    points = []
                    continue
                x = width-15-((latest-tick) & 0xffffffff)/60000*(width-60)
                y = height-22-(row[axis]-low)/(high-low)*(height-65)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)
