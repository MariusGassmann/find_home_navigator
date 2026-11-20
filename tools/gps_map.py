"""Blank north-up GNSS plot. Coordinates come from STM32 telemetry, not NMEA decoding."""
from collections import deque
import math
import time

METRES_PER_DEGREE = math.pi*6371008.8/180


class GPSMap:
    def __init__(self):
        self.home = self.position = None
        self.trail = deque(maxlen=1500)
        self.status = 'Waiting for a GNSS fix'
        self.last_data = None
        self.gap = True
        self.good_quality = False

    def navigation(self, state, values):
        if values['home']:
            self.home = (values['home_lat_e7']/1e7, values['home_lon_e7']/1e7)
        else:
            self.home = None
        if state == 'SLEEPING':
            self.status = 'SLEEPING'; self.gap = True
        elif self.status == 'SLEEPING':
            self.status = 'Waiting for a fresh GNSS fix'; self.gap = True

    def update(self, line):
        try:
            tag, tick, state, *fields = line.strip().split(',')
            if tag != 'GNSS' or not 0 <= int(tick) <= 0xffffffff:
                return False
            if state not in ('FIX','NO_FIX','STALE','NO_DATA'):
                return False
            if state != 'FIX':
                self.status = state; self.gap = True
                return True
            values = dict(field.split('=',1) for field in fields)
            lat, lon = int(values['lat_e7']), int(values['lon_e7'])
            age = int(values['age_ms'])
            if not -900000000 <= lat <= 900000000 or not -1800000000 <= lon <= 1800000000 or age < 0:
                return False
            if age > 2000:
                self.status = 'STALE'; self.gap = True
                return True
            quality = int(values.get('sat',0)) >= 6 and 0 < int(values.get('hdop_x100',0)) <= 250
        except (ValueError,KeyError):
            return False
        now = time.monotonic()
        if self.last_data is not None and now-self.last_data > 2.5:
            self.gap = True
        position = (lat/1e7,lon/1e7)
        if self.gap and self.trail and self.trail[-1] is not None:
            self.trail.append(None)  # Never invent a path through sleep or fix loss.
        if self.gap or position != self.position:
            self.trail.append(position)
        self.position, self.last_data, self.gap = position, now, False
        self.good_quality = quality
        self.status = 'FIX' if quality else 'FIX (below navigation quality limits)'
        return True

    @staticmethod
    def local(position, origin):
        lat, lon = position; lat0, lon0 = origin
        delta_lon = (lon-lon0+180) % 360-180
        return (delta_lon*METRES_PER_DEGREE*max(1e-6,math.cos(math.radians(lat0))),
                (lat-lat0)*METRES_PER_DEGREE)

    def draw(self, canvas):
        canvas.delete('all')
        w, h = max(200,canvas.winfo_width()), max(200,canvas.winfo_height())
        stale = self.gap or self.last_data is None or time.monotonic()-self.last_data > 2.5
        status = self.status if self.status != 'FIX' or not stale else 'STALE'
        canvas.create_text(w/2,15,text='GNSS map - north up | '+status,fill='white')
        origin = self.home or next((p for p in self.trail if p is not None),None)
        if origin is None:
            canvas.create_text(w/2,h/2,text='Waiting for valid GNSS coordinates',fill='#ffcc66')
            return
        points = [self.local(p,origin) for p in self.trail if p is not None]
        if self.home: points.append((0,0))
        if self.position: points.append(self.local(self.position,origin))
        xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
        ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
        margin = max(10,.1*max(xmax-xmin,ymax-ymin))
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
        left, right, top, bottom = 85,w-30,40,h-85
        scale = min((right-left)/(xmax-xmin+2*margin),(bottom-top)/(ymax-ymin+2*margin))

        def pixel(p):
            x,y = self.local(p,origin)
            return (left+right)/2+(x-cx)*scale,(top+bottom)/2-(y-cy)*scale

        # Label actual longitude/latitude; keep equal physical scale on both axes.
        for f in (0,.5,1):
            x, y = left+f*(right-left), bottom-f*(bottom-top)
            east = cx+(x-(left+right)/2)/scale
            north = cy+((top+bottom)/2-y)/scale
            lon = (origin[1]+east/(METRES_PER_DEGREE*max(1e-6,math.cos(math.radians(origin[0]))))+180)%360-180
            lat = origin[0]+north/METRES_PER_DEGREE
            canvas.create_line(x,top,x,bottom,fill='#2b3d52')
            canvas.create_line(left,y,right,y,fill='#2b3d52')
            canvas.create_text(x,bottom+15,text=f'{lon:.6f}',fill='#b9c8d9')
            canvas.create_text(left-6,y,anchor='e',text=f'{lat:.6f}',fill='#b9c8d9')
        canvas.create_text((left+right)/2,bottom+34,text='Longitude [deg] -> east',fill='white')
        canvas.create_text(12,(top+bottom)/2,angle=90,text='Latitude [deg] -> north',fill='white')
        segment = []
        for point in (*self.trail,None):
            if point is None:
                if len(segment) >= 4: canvas.create_line(*segment,fill='#66bfcc',width=2)
                segment = []
            else: segment.extend(pixel(point))
        if self.home:
            x,y = pixel(self.home); r = 5*scale
            canvas.create_oval(x-r,y-r,x+r,y+r,outline='#80e69b',dash=(4,4))
            canvas.create_text(x+12,y-12,text='H',fill='#80e69b',font=('Consolas',13,'bold'))
        if self.position:
            x,y = pixel(self.position)
            color = '#8290a0' if stale else '#66bfcc' if self.good_quality else '#ffcc66'
            canvas.create_oval(x-5,y-5,x+5,y+5,fill=color,outline='white')
        canvas.create_text(w/2,h-20,text='H: home | dashed ring: 5 m | cyan: recorded trail | grey: last fix\n'
                           'No positions are recorded while the MCU sleeps.',fill='#b9c8d9')
