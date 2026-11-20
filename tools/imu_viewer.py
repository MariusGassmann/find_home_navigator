"""IMU/compass viewer with raw logging, simulation and replay. See README.md."""
import argparse
import json
from pathlib import Path
import queue
import threading
import time

from imu_model import BOARD_AXES, Attitude, Demo, euler, norm, parse, rotate
from imu_calibration import load as load_accel_calibration
from compass_view import Compass
from fusion_view import Fusion
from navigation_view import Navigation
from gps_map import GPSMap


def serial_reader(port, lines, commands, stop, baud=115200):
    import serial
    try:
        with serial.Serial(port, baud, timeout=.1) as device:
            buffer = b''
            while not stop.is_set():
                while not commands.empty():
                    device.write(commands.get_nowait())
                raw = device.read_until(b'\n', 256)
                # Preserve partial serial lines across read timeouts.
                buffer += raw
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        lines.put_nowait(line.decode('ascii', errors='replace'))
                    except queue.Full:
                        pass  # Sequence discontinuity invalidates the estimator.
                buffer = buffer[-512:]
    except (serial.SerialException, OSError) as error:
        try:
            lines.put_nowait(f'# ERROR serial: {error}')
        except queue.Full:
            pass


class Session:
    def __init__(self, args):
        self.args = args
        calibration = load_accel_calibration(args.accel_cal) if args.accel_cal else None
        self.model = Attitude(args.axes, 0 if args.gyro_only else 1.5, not args.no_bias, calibration)
        self.queue = queue.Queue(maxsize=2048)
        self.commands = queue.Queue()
        self.stop = threading.Event()
        self.reader = None
        self.started = time.monotonic()
        self.last_data = None
        self.samples = self.bad = 0
        magnetic_cal = None
        if getattr(args, 'mag_cal', None):
            from mag_calibration import load
            magnetic_cal = load(args.mag_cal)
        self.compass = Compass(magnetic_cal, getattr(args, 'mag_axes', None))
        self.fusion = Fusion()
        self.navigation = Navigation()
        self.gps_map = GPSMap()
        self.demo = Demo() if args.demo else None
        self.replay = None
        self.log = None
        if args.log:
            path = Path(args.log)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log = path.open('w' if getattr(args, 'overwrite', False) else 'x', encoding='utf-8', buffering=1)
            self.log.write('# SETTINGS ' + json.dumps(vars(args)) + '\n')
            if calibration:
                self.log.write('# ACCEL_CAL ' + json.dumps(calibration) + '\n')
            if magnetic_cal:
                self.log.write('# MAG_CAL ' + json.dumps(magnetic_cal) + '\n')
        if args.port:
            self.reader = threading.Thread(target=serial_reader,
                                           args=(args.port, self.queue, self.commands, self.stop,
                                                 args.baud or (460800 if args.compass else 115200)), daemon=True)
            self.reader.start()
        if args.replay:
            self.replay = Path(args.replay).open(encoding='utf-8')
        if args.demo:
            self.reference()

    def reference(self):
        if self.navigation.state in ('SLEEP_NOTICE','SLEEPING'):
            return
        self.consume('# REFERENCE')
        if self.args.port:
            self.commands.put(b'r')

    def matrix_test(self):
        if self.args.port:
            self.commands.put(b't')

    def consume(self, line):
        if self.log:
            self.log.write(line.rstrip() + '\n')
        if line.startswith('# REFERENCE'):
            self.model.reference()
            return
        if line.startswith('#'):
            if line.startswith('# SENSOR '):
                self.gps_map = GPSMap()
            print(line, flush=True)
            if 'ERROR' in line:
                self.model.ready = self.model.calibrating = False
                self.model.message = line
            return
        if line.startswith('MAG,'):
            if not self.compass.update(line):
                self.bad += 1
            return
        if line.startswith('FUSE,'):
            if not self.fusion.update(line):
                self.bad += 1
            return
        if line.startswith('NAV,'):
            if not self.navigation.update(line):
                self.bad += 1
            else:
                self.gps_map.navigation(self.navigation.state,self.navigation.values)
            return
        if line.startswith('GNSS,'):
            if not self.gps_map.update(line):
                self.bad += 1
            return
        if line.startswith('RX,') or not line.strip():
            return
        sample = parse(line)
        if sample is None:
            self.bad += 1
            return
        self.model.update(sample)
        self.samples += 1
        self.last_data = time.monotonic()

    def pump(self):
        if self.demo:
            target = int((time.monotonic()-self.started)*60)
            while self.demo.sequence <= target:
                self.consume(self.demo.line())
        elif self.replay:
            # Replay at nominal 60 Hz; sample timestamps drive integration.
            target = int((time.monotonic()-self.started)*60)
            while self.samples <= target:
                line = self.replay.readline()
                if not line:
                    return False
                self.consume(line.strip())
        else:
            for _ in range(256):
                try:
                    self.consume(self.queue.get_nowait())
                except queue.Empty:
                    break
        return True

    def summary(self):
        m = self.model
        age = time.monotonic()-self.last_data if self.last_data is not None else float('inf')
        magnetic_heading, magnetic_status = self.compass.heading(m.accel, m.last.tick if m.last else None, age, m.up_vector())
        return dict(samples=self.samples, malformed=self.bad, gaps=m.gaps, ready=m.ready,
                    elapsed_s=round(m.elapsed, 3), roll_pitch_yaw_deg=euler(m.q),
                    accumulated_yaw_deg=m.yaw_total, measured_bias_dps=m.bias,
                    subtract_bias=m.subtract_bias, gravity_gain=m.gain, axes=self.args.axes,
                    accel_offset_g=m.accel_offset, accel_scale=m.accel_scale,
                    mag_axes=self.compass.axes, magnetic_heading_deg=magnetic_heading,
                    magnetic_heading_status=magnetic_status,
                    firmware_fusion_state=self.fusion.state,
                    firmware_gyro_compass_fused_deg=self.fusion.angles,
                    navigator_state=self.navigation.state, navigator=self.navigation.values,
                    compass_samples=self.compass.samples, compass_overflows=self.compass.overflows,
                    status=m.message)

    def close(self):
        self.stop.set()
        if self.reader:
            self.reader.join(timeout=1)
        if self.replay:
            self.replay.close()
        if self.log:
            self.log.close()
        summary = json.dumps(self.summary(), indent=2)
        if self.args.log:
            Path(self.args.log).with_suffix('.summary.json').write_text(summary, encoding='utf-8')
        print(summary)


def gui(session):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('Find Home | Enter = home / sleep; sensor bench = level reference')
    height = min(1050 if session.compass.axes else 760,root.winfo_screenheight()-80)
    root.geometry(f'{1100 if session.compass.axes else 1000}x{height}')
    controls = ttk.Frame(root, padding=10)
    controls.pack(fill='x')
    action = ttk.Button(controls, text='Start / reference (Enter)', command=session.reference)
    action.pack(side='left')
    ttk.Label(controls, text=f'  {"SIMULATION" if session.args.demo else "REPLAY" if session.args.replay else session.args.port}  |  Laptop reference: {session.args.axes}  |  '
              f'{"gyro only" if session.args.gyro_only else "gravity corrected"}  |  '
              f'bias subtraction {"OFF" if session.args.no_bias else "ON"}  |  '
              f'accel cal {"ON" if session.args.accel_cal else "OFF"}').pack(side='left')
    map_window = map_canvas = None

    def open_map():
        nonlocal map_window, map_canvas
        if map_window is not None and map_window.winfo_exists():
            map_window.lift(); return
        map_window = tk.Toplevel(root)
        map_window.title('Find Home - GNSS map')
        map_window.geometry('640x480')
        map_canvas = tk.Canvas(map_window,bg='#142032',highlightthickness=0)
        map_canvas.pack(fill='both',expand=True)
    if session.args.compass:
        ttk.Button(controls,text='GNSS map',command=open_map).pack(side='right')
    state = tk.StringVar()
    ttk.Label(root, textvariable=state, padding=8).pack(fill='x')
    # Keep diagnostics accessible on laptops with display scaling/small screens.
    viewport = tk.Canvas(root, highlightthickness=0)
    scrollbar = ttk.Scrollbar(root, orient='vertical', command=viewport.yview)
    scrollbar.pack(side='right', fill='y')
    viewport.pack(fill='both', expand=True)
    viewport.configure(yscrollcommand=scrollbar.set)
    body = ttk.Frame(viewport)
    content = viewport.create_window(0, 0, anchor='nw', window=body)
    body.bind('<Configure>', lambda _: viewport.configure(scrollregion=viewport.bbox('all')))
    viewport.bind('<Configure>', lambda e: viewport.itemconfigure(content, width=e.width))
    root.bind('<MouseWheel>', lambda e: viewport.yview_scroll(-int(e.delta/120), 'units'))
    navigation = tk.Canvas(body, bg='#142032', height=110, highlightthickness=0)
    canvas = tk.Canvas(body, bg='#142032', height=280, highlightthickness=0)
    canvas.pack(fill='x')
    magnetic_frame = ttk.Frame(body)
    magnetic = tk.Canvas(magnetic_frame, bg='#142032', height=240, highlightthickness=0)
    dial = tk.Canvas(magnetic_frame, bg='#142032', width=280, height=240, highlightthickness=0)
    magnetic_text = tk.StringVar()
    fusion = tk.Canvas(body, bg='#142032', height=145, highlightthickness=0)
    if session.args.compass:
        magnetic_frame.pack(fill='x')
        if session.compass.axes:
            dial.pack(side='right', fill='y')
        magnetic.pack(side='left', fill='both', expand=True)
        ttk.Label(body, textvariable=magnetic_text, padding=4).pack(fill='x')
        fusion.pack(fill='x')
    readings = tk.StringVar()
    ttk.Label(body, textvariable=readings, font=('Consolas', 11), padding=10).pack(fill='x')
    ttk.Label(body, text='Board +X points toward POTI. Yaw is relative to start. '
              '3D: relative gyro/gravity. Dial: home in navigator, north in bench. Comparison: magnetic headings. Field plots: sensor axes.'
              if session.args.compass else
              'Board +X points toward POTI. Yaw is relative to start. T: LED test; Enter: reference.', padding=8).pack(fill='x')
    root.bind('<Return>', lambda _: session.reference())
    root.bind('<KeyPress-t>', lambda _: session.matrix_test())
    root.bind('<KeyPress-T>', lambda _: session.matrix_test())
    root.bind('<Escape>', lambda _: root.destroy())

    def draw():
        running = session.pump()
        m = session.model
        bias_ms = session.fusion.bias_ms
        source = 'STM32'
        if bias_ms is None:
            source = 'Laptop reference'
            bias_ms = 2000 if m.ready else min(2000,((m.last.tick-m.window[0].tick)&0xffffffff)) if m.window else 0
        calibration_status = (f' | {source} IMU calibration: {bias_ms/1000:.1f}/2 s'
                              if bias_ms < 2000 else '')
        if map_window is not None and map_window.winfo_exists():
            session.gps_map.draw(map_canvas)
        stale = session.last_data is None or time.monotonic()-session.last_data > .5
        state.set(('NO FRESH DATA — ' if stale else '') + m.message)
        if session.navigation.state is not None:
            if not navigation.winfo_manager():
                navigation.pack(fill='x',before=canvas)
            session.navigation.draw(navigation)
            action.configure(text=session.navigation.button_text(),
                             state='disabled' if session.navigation.state in ('SLEEP_NOTICE','SLEEPING') else 'normal')
            state.set('Navigator: '+session.navigation.state+' | STM32 heading: '+str(session.fusion.state)+
                      '\nLaptop IMU: '+m.message)
        state.set(state.get()+calibration_status)
        if session.args.compass:
            session.fusion.draw(fusion)
            session.compass.draw(magnetic)
            age = time.monotonic()-session.last_data if session.last_data is not None else float('inf')
            up = m.up_vector()
            if up is None and .9 <= norm(m.accel) <= 1.1:
                up = tuple(v/norm(m.accel) for v in m.accel)
            compass_angle, compass_reason = session.compass.heading(
                m.accel, m.last.tick if m.last else None, age, up)
            magnetic_text.set(session.compass.text() +
                              (f' | board axes: {session.compass.axes}'
                               if session.compass.axes else '')+
                              '\nCompass diagnostic: '+
                              (f'{compass_angle:.1f} deg CW | ' if compass_angle is not None else '')+
                              compass_reason+(' | accelerometer tilt only' if not m.ready else ''))
            if session.compass.axes:
                age = time.monotonic()-session.last_data if session.last_data is not None else float('inf')
                target = session.navigation.heading() if session.navigation.state is not None else (
                    session.fusion.heading() if session.fusion.state is not None else None)
                session.compass.draw_heading(dial, m.accel, m.last.tick if m.last else None, age,
                                            m.up_vector(),target,'H' if session.navigation.state else 'N')
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        scale = min(width, height)*.47

        def project(v):
            x, y, z = v
            depth = .514*x-.686*y+.514*z
            perspective = 4/(4-depth)
            return (width/2 + scale*(.8*x+.6*y)*perspective,
                    height/2 - scale*(-.309*x+.412*y+.857*z)*perspective)

        for i in range(-4, 5):
            for a, b in [((i*.3, -1.2, -.12), (i*.3, 1.2, -.12)),
                         ((-1.2, i*.3, -.12), (1.2, i*.3, -.12))]:
                canvas.create_line(*project(a), *project(b), fill='#2b3d52')
        vertices = [(x, y, z) for z in (-.035, .035) for y in (-.35, .35) for x in (-.5, .5)]
        display_q = m.q
        points = [rotate(display_q, v) for v in vertices]
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                 (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        depth = lambda face: sum(.514*points[i][0]-.686*points[i][1]+.514*points[i][2] for i in face)
        for face in sorted(faces, key=depth):
            color = '#287fa4' if 4 in face and 7 in face else '#17536d'
            canvas.create_polygon(*(c for i in face for c in project(points[i])),
                                  fill=color, outline='#86c9df', width=2)
        for v, label, color in [((.85, 0, 0), '+X forward', '#ff7979'),
                                 ((0, .65, 0), '+Y left', '#80e69b'),
                                 ((0, 0, .65), '+Z up', '#8dabff')]:
            end = project(rotate(display_q, v))
            canvas.create_line(*project((0, 0, 0)), *end, fill=color, width=3, arrow='last')
            canvas.create_text(end[0], end[1]-15, text=label, fill=color)
        canvas.create_text(*project(rotate(display_q, (.3, 0, .04))), text='POTI', fill='white')
        canvas.create_text(*project(rotate(display_q, (-.3, 0, .04))), text='BUTTONS', fill='white')
        canvas.create_text(12, 15, anchor='w', fill='white', text=(
            'Laptop gyro/gravity orientation' if m.ready else
            m.message))
        roll, pitch, yaw = euler(display_q)
        rate = m.yaw_total/m.elapsed*60 if m.elapsed else 0
        readings.set(f'Roll {roll:8.2f}°   Pitch {pitch:8.2f}°   Yaw {yaw:8.2f}°   elapsed {m.elapsed:7.1f} s\n'
                     f'Accumulated yaw {m.yaw_total:8.2f}°   average {rate:8.2f}°/min (drift only while stationary)\n'
                     f'accel [g]   {m.accel[0]:8.4f} {m.accel[1]:8.4f} {m.accel[2]:8.4f}   norm {norm(m.accel):.4f}   raw norm {norm(m.raw_accel):.4f}\n'
                     f'gyro [°/s]  {m.gyro[0]:8.3f} {m.gyro[1]:8.3f} {m.gyro[2]:8.3f}\n'
                     f'bias [°/s]  {m.bias[0]:8.3f} {m.bias[1]:8.3f} {m.bias[2]:8.3f}   gaps {m.gaps}   bad lines {session.bad}')
        if session.args.seconds and time.monotonic()-session.started >= session.args.seconds:
            root.destroy()
        elif running:
            root.after(16, draw)

    root.after(16, draw)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--port', help='ST-LINK virtual COM port, e.g. COM5')
    source.add_argument('--demo', action='store_true')
    source.add_argument('--replay', help='Replay a raw capture; use its original --axes/mode flags')
    parser.add_argument('--axes', default=BOARD_AXES, help='Signed sensor axes for board X,Y,Z; default: %(default)s. Use --axes=... for a leading minus.')
    parser.add_argument('--compass', action='store_true', help='Show raw compass alongside IMU; USB defaults to 460800')
    parser.add_argument('--mag-axes', help='Enable magnetic heading with signed sensor axes for board XYZ; inferred mounting: y,x,-z')
    parser.add_argument('--mag-cal', help='Apply validated sensor-frame compass calibration JSON')
    parser.add_argument('--baud', type=int, help='Override USB serial baud rate')
    parser.add_argument('--gyro-only', action='store_true')
    parser.add_argument('--no-bias', action='store_true')
    parser.add_argument('--accel-cal', help='Apply sensor-frame accelerometer calibration JSON; default OFF')
    parser.add_argument('--overwrite', action='store_true', help='Explicitly replace an existing capture')
    parser.add_argument('--log', help='Raw text capture including reference events')
    parser.add_argument('--headless', action='store_true', help='Capture/estimate without a window')
    parser.add_argument('--seconds', type=float, default=0, help='Stop after N seconds; 0 means until closed')
    args = parser.parse_args()
    if args.log and Path(args.log).exists() and not args.overwrite:
        parser.error('Log already exists; choose a new filename or use --overwrite')
    if args.mag_axes and not (args.compass and args.mag_cal):
        parser.error('--mag-axes requires --compass and --mag-cal')
    if args.mag_cal and not args.compass:
        parser.error('--mag-cal requires --compass')
    if args.seconds < 0:
        parser.error('--seconds must be nonnegative')
    if args.replay and args.log and Path(args.replay).resolve() == Path(args.log).resolve():
        parser.error('replay input and log output must differ')
    session = Session(args)
    try:
        if args.headless:
            while session.pump():
                if args.seconds and time.monotonic()-session.started >= args.seconds:
                    break
                time.sleep(.005)
        else:
            gui(session)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == '__main__':
    main()
