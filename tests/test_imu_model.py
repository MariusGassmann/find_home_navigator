import itertools
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from imu_model import Attitude, Demo, Sample, advance, euler, mounting, norm, parse, rotate


def sample(i, accel=(0, 0, 1), gyro=(0, 0, 0), reference=0, start=0):
    return Sample((start+round(i*1000/60)) & 0xffffffff, i & 0xffffffff, reference, accel, gyro)


def calibrated(**kwargs):
    model = Attitude(**kwargs)
    model.reference()
    for i in range(121):
        model.update(sample(i))
    assert model.ready
    return model


class ProtocolTests(unittest.TestCase):
    def test_decode_units_and_limits(self):
        s = parse('IMU,4294967295,9,1,-32768,0,32767,100,-100,0\r\n')
        self.assertAlmostEqual(s.accel[0], -1.998848)
        for actual, expected in zip(s.gyro, (1.75, -1.75, 0)):
            self.assertAlmostEqual(actual, expected)

    def test_bad_packets(self):
        for line in ['', '# diagnostic', 'IMU,0,0,0,0,0,1,0,0',
                     'IMU,0,0,0,0,0,32768,0,0,0', 'IMU,-1,0,0,0,0,1,0,0,0',
                     'IMU,0,0,0,0,0,nan,0,0,0', 'IMU,4294967296,0,0,0,0,1,0,0,0']:
            self.assertIsNone(parse(line), line)


class OrientationTests(unittest.TestCase):
    def test_all_24_proper_mountings(self):
        count = 0
        for axes in itertools.permutations('xyz'):
            for signs in itertools.product(('', '-'), repeat=3):
                spec = ','.join(s+a for s, a in zip(signs, axes))
                try:
                    transform = mounting(spec)
                except ValueError:
                    continue
                count += 1
                self.assertAlmostEqual(norm(transform((1, 2, 3))), math.sqrt(14))
        self.assertEqual(count, 24)
        for bad in ['x,x,z', 'x,y,-z', 'a,b,c', 'x,y', '']:
            with self.assertRaises(ValueError):
                mounting(bad)

    def test_yaw_wrap_accumulation(self):
        model = calibrated(gain=0)
        for i in range(121, 1561):
            model.update(sample(i, gyro=(0, 0, 30)))
        self.assertAlmostEqual(model.yaw_total, 720, places=5)

    def test_bias_and_raw_drift(self):
        for subtract in (True, False):
            model = Attitude(subtract_bias=subtract)
            model.reference()
            for i in range(3721):
                model.update(sample(i, gyro=(0, 0, .2)))
            self.assertAlmostEqual(model.bias[2], .2)
            self.assertAlmostEqual(model.yaw_total, 0 if subtract else 12, places=5)

    def test_gravity_correction_converges(self):
        model = calibrated()
        model.q = advance(model.q, (math.radians(20), 0, 0), 1)
        for i in range(121, 721):
            model.update(sample(i))
        self.assertLess(abs(euler(model.q)[0]), .001)

    def test_large_acceleration_rejected(self):
        model = calibrated()
        model.update(sample(121, accel=(.5, 0, 1.3)))
        self.assertEqual(model.q, (1, 0, 0, 0))
        self.assertIn('gyro propagation', model.message)

    def test_tilted_motion_tracks_gravity(self):
        model = calibrated()
        q = (1, 0, 0, 0)
        for i in range(121, 481):
            rates = (10*math.sin(i*.02), 8*math.cos(i*.02), 15)
            q = advance(q, tuple(map(math.radians, rates)), .017 if i % 3 != 2 else .016)
            a = rotate((q[0], *(-x for x in q[1:])), (0, 0, 1))
            model.update(sample(i, accel=a, gyro=rates))
        dot = abs(sum(x*y for x, y in zip(q, model.q)))
        self.assertGreater(dot, .9999)


class CalibrationTests(unittest.TestCase):
    def test_must_start_explicitly(self):
        model = Attitude()
        for i in range(200):
            model.update(sample(i))
        self.assertFalse(model.ready)

    def test_button_reference_and_repeated_reference(self):
        model = calibrated()
        model.update(sample(121, reference=1))
        self.assertFalse(model.ready)
        self.assertTrue(model.calibrating)
        for i in range(122, 242):
            model.update(sample(i, reference=1))
        self.assertTrue(model.ready)

    def test_tilt_motion_and_noise_fail(self):
        for a, g in [((1, 0, 0), (0, 0, 0)), ((0, 0, -1), (0, 0, 0)),
                     ((0, 0, 1), (5, 0, 0)), ((0, 0, 0), (0, 0, 0))]:
            model = Attitude()
            model.reference()
            for i in range(200):
                model.update(sample(i, accel=a, gyro=g))
            self.assertFalse(model.ready)
        model = Attitude()
        model.reference()
        for i in range(121):
            model.update(sample(i, gyro=(0, 0, (-1)**i)))
        self.assertFalse(model.ready)
        self.assertIn('noise', model.message)

    def test_gap_duplicate_and_restart_invalidate(self):
        for s in [sample(122), sample(120), sample(0),
                  Sample(2200, 121, 0, (0, 0, 1), (0, 0, 0))]:
            model = calibrated()
            model.update(s)
            self.assertFalse(model.ready)
            self.assertEqual(model.gaps, 1)

    def test_timestamp_and_sequence_wrap(self):
        model = Attitude()
        model.reference()
        for i in range(140):
            model.update(sample(i, start=0xfffffc00))
        self.assertTrue(model.ready)
        self.assertEqual(model.gaps, 0)
        model.last = Sample(100, 0xffffffff, 0, (0, 0, 1), (0, 0, 0))
        model.update(Sample(117, 0, 0, (0, 0, 1), (0, 0, 0)))
        self.assertTrue(model.ready)

    def test_saturation_invalidates(self):
        model = calibrated()
        model.update(sample(121, gyro=(0, 0, 499)))
        self.assertFalse(model.ready)

    def test_demo_is_valid_and_tracks(self):
        demo, model = Demo(), Attitude()
        model.reference()
        for _ in range(1800):
            s = parse(demo.line())
            self.assertIsNotNone(s)
            model.update(s)
        self.assertTrue(model.ready)
        self.assertEqual(model.gaps, 0)
        self.assertGreater(abs(sum(x*y for x, y in zip(demo.q, model.q))), .999)


if __name__ == '__main__':
    unittest.main()
