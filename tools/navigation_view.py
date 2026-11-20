"""Read and display navigator state, position and distance telemetry."""
import time

STATES = ('NO_HOME','CAPTURING','SLEEP_NOTICE','SLEEPING','NO_FIX','NO_HEADING','GO_HOME','COASTING','ARRIVED')


class Navigation:
    def __init__(self):
        self.state = None
        self.values = {}
        self.last_data = None

    def update(self, line):
        try:
            tag, tick, state, *fields = line.strip().split(',')
            values = dict(field.split('=',1) for field in fields)
            if len(values) != len(fields):
                return False
            values = {key:int(value) for key,value in values.items()}
            required = {'home_lat_e7','home_lon_e7','lat_e7','lon_e7','distance_cm','bearing_mdeg',
                        'relative_mdeg','samples','sat','hdop_x100','home','declination_mdeg','red','green'}
            if tag != 'NAV' or state not in STATES or not 0 <= int(tick) <= 0xffffffff or not required <= values.keys():
                return False
            if (not 0 <= values['red'] <= 0xfff or not 0 <= values['green'] <= 0xfff or
                    not 0 <= values['samples'] <= 10 or values['home'] not in (0,1) or
                    not 0 <= values['sat'] <= 99 or not 0 <= values['hdop_x100'] <= 99999 or
                    not -900000000 <= values['lat_e7'] <= 900000000 or
                    not -1800000000 <= values['lon_e7'] <= 1800000000):
                return False
        except (ValueError,KeyError):
            return False
        self.state, self.values, self.last_data = state, values, time.monotonic()
        return True

    def button_text(self):
        if self.state == 'NO_HOME': return 'Set home (Enter)'
        if self.state == 'CAPTURING': return 'Cancel capture (Enter)'
        if self.state in ('SLEEP_NOTICE','SLEEPING'): return 'Wake with board Enter'
        return 'Sleep (Enter)'

    def heading(self):
        if self.last_data is None or time.monotonic()-self.last_data > 1.5:
            return None, 'No fresh navigation data'
        if self.state not in ('GO_HOME','COASTING'):
            return None, self.state
        # Dial expects CCW direction of its target relative to forward.
        return self.values['relative_mdeg']/1000 % 360, 'Direction toward home'

    def draw(self, canvas):
        canvas.delete('all')
        if self.state is None:
            return
        v = self.values
        age = time.monotonic()-self.last_data
        stale = age > 1.5 and self.state != 'SLEEPING'
        status = self.state + (' (telemetry stale)' if stale else '')
        distance = '--' if v['distance_cm'] < 0 else f'{v["distance_cm"]/100:.1f} m'
        text = (f'Navigator: {status}   distance {distance}   home samples {v["samples"]}/10\n'
                f'Home: {v["home_lat_e7"]/1e7:.7f}, {v["home_lon_e7"]/1e7:.7f}'
                if v['home'] else f'Navigator: {status}   home samples {v["samples"]}/10\nHome: not saved')
        text += (f'\nPosition: {v["lat_e7"]/1e7:.7f}, {v["lon_e7"]/1e7:.7f}'
                 f'   sat {v["sat"]}   HDOP {v["hdop_x100"]/100:.2f}'
                 f'   declination {v["declination_mdeg"]/1000:+.1f} deg E')
        canvas.create_text(12,12,anchor='nw',text=text,fill='white',font=('Consolas',10))
