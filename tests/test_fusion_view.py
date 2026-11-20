from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from export_calibration import generate
from fusion_view import Fusion, MISSING
from imu_viewer import Session

ROOT = Path(__file__).resolve().parents[1]


class FusionTests(unittest.TestCase):
    def test_mixed_stream_wrap_faults_and_freshness(self):
        args = SimpleNamespace(accel_cal=None, axes='y,-x,z', gyro_only=False,
                               no_bias=False, demo=False, log=None, port=None, replay=None)
        session = Session(args)
        for tick, angle in ((100,359000),(200,1000),(300,3000)):
            session.consume(f'IMU,{tick},{tick},0,0,0,16393,0,0,0')
            session.consume(f'MAG,{tick},0,0,1125,1500,1')
            session.consume(f'FUSE,{tick},TRACKING,{angle},{angle},{angle},0,1')
        f = session.fusion
        self.assertEqual(session.bad,0)
        self.assertEqual([row[1][2] for row in f.history],[359,361,363])
        self.assertEqual(f.heading()[0],3)
        self.assertTrue(f.update(f'FUSE,900,COASTING,3000,{MISSING},3000,0,0'))
        self.assertIn('fallback',f.heading()[1])
        self.assertIsNone(f.angles[1])
        self.assertTrue(f.update('FUSE,1000,UNRELIABLE,3000,3000,3000,0,0'))
        self.assertIsNone(f.heading()[0])

        f.last_data -= 1
        self.assertIn('fresh',f.heading()[1])
        for line in ('FUSE,1,BOGUS,0,0,0,0,0','FUSE,1,TRACKING,360000,0,0,0,1',
                     'FUSE,1,TRACKING,0,0,0,8,1','FUSE,-1,TRACKING,0,0,0,0,1',
                     f'FUSE,1,TRACKING,{MISSING},0,0,0,1','FUSE,nan,WAITING,0,0,0,0,0'):
            self.assertFalse(f.update(line))
        self.assertTrue(f.update(f'FUSE,10,WAITING,{MISSING},{MISSING},{MISSING},0,0'))
        self.assertFalse(f.history)
        self.assertIsNone(f.heading()[0])

    def test_onboard_bias_progress_and_old_captures(self):
        f=Fusion()
        prefix=f'FUSE,1000,CALIBRATING,{MISSING},{MISSING},{MISSING},0,0'
        for ms in (0, 750, 1900, 2000):
            self.assertTrue(f.update(prefix+f',{ms}'))
            self.assertEqual(f.bias_ms,ms)
        for suffix in (',-1',',2001',',bad',',0,0'):
            self.assertFalse(f.update(prefix+suffix))
        self.assertTrue(f.update(prefix))
        self.assertIsNone(f.bias_ms)

    def test_calibration_export_is_reproducible_and_rejects_bad_inputs(self):
        a, m = ROOT/'calibration/imu_accel.json', ROOT/'calibration/mag.json'
        originals = a.read_bytes(), m.read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'generated.h'
            generate(a,m,output)
            first = output.read_bytes()
            generate(a,m,output)
            self.assertEqual(output.read_bytes(),first)
            self.assertIn(b'north_calibration_t',first)
            bad = json.loads(m.read_text())
            bad['matrix'][0][0] = float('nan')
            path = Path(temp)/'bad.json'
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                generate(a,path,output)
            self.assertEqual(output.read_bytes(),first)
            bad['quality']['accepted'] = False
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                generate(a,path,output)
        self.assertEqual((a.read_bytes(),m.read_bytes()),originals)

    def test_gui_with_firmware_fusion_and_field_plots(self):
        import tkinter as tk
        from imu_viewer import gui
        args = SimpleNamespace(accel_cal=None, axes='y,-x,z', gyro_only=False,
                               no_bias=False, demo=False, log=None, port=None, replay=None,
                               compass=True, mag_axes='y,x,-z', seconds=.1)
        session = Session(args)
        for line in ('IMU,100,1,0,0,0,16393,0,0,0', 'MAG,100,1,0,1125,1500,1',
                     'FUSE,100,TRACKING,0,1000,500,0,1'):
            session.queue.put(line)
        root = tk.Tk()
        root.withdraw()
        with patch('tkinter.Tk',return_value=root):
            gui(session)
        self.assertEqual(session.fusion.angles,(0,1,.5))
        self.assertEqual(session.bad,0)
