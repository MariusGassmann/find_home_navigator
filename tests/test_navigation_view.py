from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from imu_viewer import Session, gui
from gps_map import GPSMap, METRES_PER_DEGREE


def record(state='GO_HOME', **changes):
    values = dict(home_lat_e7=473769000,home_lon_e7=85417000,lat_e7=473760000,
                  lon_e7=85417000,distance_cm=10007,bearing_mdeg=0,relative_mdeg=90000,
                  samples=10,sat=8,hdop_x100=120,home=1,declination_mdeg=0,red=1632,green=16)
    values.update(changes)
    return 'NAV,20000,'+state+','+','.join(f'{k}={v}' for k,v in values.items())


class NavigationTests(unittest.TestCase):
    def session(self):
        return Session(SimpleNamespace(accel_cal=None,axes='y,-x,z',gyro_only=False,no_bias=False,
                                       demo=False,log=None,port=None,replay=None,compass=True,
                                       mag_axes='y,x,-z',seconds=.1))

    def test_protocol_and_actions_across_sleep(self):
        s=self.session()
        s.consume(record())
        self.assertEqual(s.navigation.state,'GO_HOME')
        self.assertEqual(s.navigation.heading()[0],90)
        self.assertEqual(s.summary()['navigator']['distance_cm'],10007)
        self.assertEqual(s.bad,0)
        for state in ('SLEEP_NOTICE','SLEEPING'):
            s.consume(record(state))
            self.assertIsNone(s.navigation.heading()[0])
            s.reference()
            self.assertFalse(s.model.calibrating)
            self.assertIn('board',s.navigation.button_text())
        s.consume(record('NO_FIX'))
        self.assertIsNone(s.navigation.heading()[0])
        s.consume(record('GO_HOME'))
        s.navigation.last_data -= 2
        self.assertIsNone(s.navigation.heading()[0])
        for bad in (record('BOGUS'),record(red=4096),record(samples=11),record(home=2),
                    record(sat=-1),'NAV,1,GO_HOME,red=0,green=0',record()+',red=1'):
            self.assertFalse(s.navigation.update(bad))

    def test_gui_shows_navigation_matrix_and_existing_plots(self):
        import tkinter as tk
        s=self.session()
        s.args.seconds=.5
        for line in ('IMU,20000,1,1,0,0,16393,0,0,0','MAG,20000,1,0,1125,1500,1',
                     'FUSE,20000,TRACKING,90000,90000,90000,2,1',record(),
                     'GNSS,20000,FIX,lat_e7=473760000,lon_e7=85417000,age_ms=0,sat=8,hdop_x100=120'):
            s.queue.put(line)
        root=tk.Tk(); root.withdraw()
        errors=[]
        diagnostics=[]
        root.report_callback_exception=lambda *error: errors.append(error)
        def open_map():
            # Arrival must not hide the independent heading/sensor diagnostics.
            s.queue.put(record('ARRIVED',distance_cm=200))
            for control in root.winfo_children()[0].winfo_children():
                if control.cget('text') == 'GNSS map': control.invoke()
            for window in root.winfo_children():
                if isinstance(window,tk.Toplevel): window.withdraw()
        def inspect():
            def visit(widget):
                if isinstance(widget,tk.Canvas):
                    diagnostics.extend(widget.itemcget(i,'text') for i in widget.find_all()
                                       if widget.type(i)=='text')
                elif widget.winfo_class()=='TLabel':
                    diagnostics.append(widget.cget('text'))
                for child in widget.winfo_children(): visit(child)
            visit(root)
        root.after(25,open_map)
        root.after(150,inspect)
        with patch('tkinter.Tk',return_value=root):
            gui(s)
        self.assertFalse(errors,errors)
        self.assertEqual(s.navigation.state,'ARRIVED')
        text='\n'.join(diagnostics)
        for required in ('STM32 heading: TRACKING','Compass diagnostic:',
                         'Place board level; press Enter to reference','Roll','Fused'):
            self.assertIn(required,text)
        self.assertEqual(s.bad,0)
        self.assertEqual(s.gps_map.position,(47.376,8.5417))

    def test_map_coordinates_validity_and_gaps(self):
        m=GPSMap()
        def fix(lat=473769000,lon=85417000,age=0):
            return f'GNSS,1000,FIX,lat_e7={lat},lon_e7={lon},age_ms={age},sat=8,hdop_x100=120'
        self.assertTrue(m.update('GNSS,0,NO_FIX'))
        self.assertIsNone(m.position)  # Never plot the uninitialized (0,0).
        self.assertTrue(m.update(fix()))
        self.assertEqual(m.position,(47.3769,8.5417))
        m.update(fix())
        self.assertEqual(len(m.trail),1)
        m.navigation('SLEEPING',dict(home=1,home_lat_e7=473769000,home_lon_e7=85417000))
        self.assertTrue(m.gap)
        self.assertEqual(m.home,m.position)
        m.update(fix(lat=473770000))
        self.assertEqual(list(m.trail)[1],None)
        self.assertEqual(len(m.trail),3)
        m.update('GNSS,2000,NO_FIX')
        last=m.position
        m.update(fix(lat=473771000,age=2001))
        self.assertEqual(m.position,last)
        self.assertTrue(m.gap)
        for bad in (fix(lat=900000001),fix(lon=-1800000001),fix(age=-1),'GNSS,1,FIX',
                    'GNSS,1,BOGUS','GNSS,nan,NO_FIX'):
            self.assertFalse(m.update(bad))
        north=GPSMap.local((47+5/METRES_PER_DEGREE,8),(47,8))
        self.assertAlmostEqual(north[0],0)
        self.assertAlmostEqual(north[1],5,places=6)
        east=GPSMap.local((0,-179.99999),(0,179.99999))
        self.assertAlmostEqual(east[0],2.2239016,places=5)
        for i in range(1600): m.update(fix(lat=473769000+i))
        self.assertEqual(len(m.trail),1500)
        s=self.session(); s.consume(fix())
        with patch('builtins.print'): s.consume('# SENSOR v7')
        self.assertIsNone(s.gps_map.position)
