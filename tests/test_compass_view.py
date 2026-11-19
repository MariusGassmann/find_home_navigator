from pathlib import Path
import sys
import unittest
import tempfile
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from compass_view import Compass
from imu_viewer import Session


class CombinedTests(unittest.TestCase):
    def test_scale_flags_and_malformed_packets(self):
        c = Compass()
        self.assertTrue(c.update('MAG,10,0,3750,-3750,0,25'))
        self.assertEqual(c.points[-1], (100, -100, 0))
        self.assertTrue(c.update('MAG,20,1,0,0,0,3'))
        self.assertTrue(c.update('MAG,30,2,0,0,0,0'))
        self.assertEqual((c.samples, c.overflows), (1, 1))
        for line in ('MAG,1,2,3', 'MAG,1,2,40000,0,0,1', 'MAG,1,2,0,0,0,256',
                     'MAG,1,2,nan,0,0,1'):
            self.assertFalse(c.update(line))
        self.assertEqual(c.samples, 1)

    def test_history_is_bounded_and_empty_state_is_clear(self):
        c = Compass()
        self.assertIn('waiting', c.text())
        for i in range(2000):
            c.update(f'MAG,{i},{i},10,20,30,1')
        self.assertEqual((c.samples, len(c.points)), (2000, 1500))
        c.last_data -= 1
        self.assertIn('STALE', c.text())

    def test_mixed_stream_keeps_imu_and_compass_separate(self):
        args = SimpleNamespace(accel_cal=None, axes='y,-x,z', gyro_only=False,
                               no_bias=False, demo=False, log=None, port=None, replay=None)
        s = Session(args)
        s.reference()
        for i in range(130):
            s.consume(f'IMU,{round(i*1000/60)},{i},0,0,0,16393,0,0,0')
            s.consume(f'MAG,{i*20},{i},375,750,-375,25')
            s.consume(f'RX,{i*20},{i},00')
            s.consume(f'GNSS,{i*20},NO_FIX')
        self.assertTrue(s.model.ready)
        self.assertEqual((s.samples, s.compass.samples, s.bad, s.model.gaps), (130,130,0,0))
        self.assertEqual(s.compass.points[-1], (10,20,-10))
        s.consume('MAG,broken')
        self.assertEqual(s.bad, 1)
        self.assertTrue(s.model.ready)

    def test_viewer_does_not_overwrite_existing_log(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'existing.txt'; p.write_text('original')
            args=SimpleNamespace(accel_cal=None,axes='y,-x,z',gyro_only=False,no_bias=False,
                                 demo=False,port=None,replay=None,log=str(p))
            with self.assertRaises(FileExistsError): Session(args)
            self.assertEqual(p.read_text(),'original')
