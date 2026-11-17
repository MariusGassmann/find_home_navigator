import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from imu_calibration import analyse, correct, fit, load
from imu_model import Attitude, Sample, mounting, norm


class AccelerometerCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.offset = (.012, -.021, .053)
        self.scale = (1.006, .995, .992)
        # Six deliberately imperfectly aligned holds, each exactly 1 g.
        directions = [(1, .08, -.03), (-1, .05, .09), (.03, 1, -.04),
                      (-.07, -1, .02), (.01, .04, 1), (-.03, .02, -1)]
        self.true = [tuple(x/norm(v) for x in v) for v in directions]
        self.raw = [tuple(v[i]/self.scale[i]+self.offset[i] for i in range(3)) for v in self.true]

    def test_fit_tilted_poses_recovers_offsets_and_scales(self):
        result = fit(self.raw)
        for i in range(3):
            self.assertAlmostEqual(result['offset_g'][i], self.offset[i], places=10)
            self.assertAlmostEqual(result['scale'][i], self.scale[i], places=10)
        for raw, expected in zip(self.raw, self.true):
            for actual, value in zip(correct(raw, result), expected):
                self.assertAlmostEqual(actual, value, places=10)

    def test_reject_missing_and_degenerate_poses(self):
        for vectors in [self.raw[:5], [self.raw[0]]*6]:
            with self.assertRaises(ValueError):
                fit(vectors)

    def test_reject_implausible_sensor_correction(self):
        with self.assertRaises(ValueError):
            fit([tuple(3*x for x in v) for v in self.true])

    def test_model_corrects_in_sensor_frame_before_mounting(self):
        c = dict(offset_g=self.offset, scale=self.scale)
        model = Attitude(axes='y,-x,z', accel_calibration=c)
        model.update(Sample(0, 0, 0, self.raw[0], (0, 0, 0)))
        for a, b in zip(model.accel, mounting('y,-x,z')(self.true[0])):
            self.assertAlmostEqual(a, b)
        self.assertEqual(model.raw_accel, mounting('y,-x,z')(self.raw[0]))

    def test_raw_saturation_cannot_be_hidden_by_calibration(self):
        c = dict(offset_g=(0, 0, .1), scale=(1, 1, .9))
        model = Attitude(accel_calibration=c)
        model.ready = True
        model.update(Sample(0, 0, 0, (0, 0, 1), (0, 0, 0)))
        model.update(Sample(17, 1, 0, (0, 0, 1.998), (0, 0, 0)))
        self.assertFalse(model.ready)
        self.assertIn('saturated', model.message)

    def test_load_validates_provenance_format_and_numeric_values(self):
        good = dict(schema=1, frame='sensor_xyz', who_am_i=0x71,
                    offset_g=list(self.offset), scale=list(self.scale))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cal.json'
            path.write_text(json.dumps(good))
            self.assertEqual(load(path), good)
            for changes in [dict(frame='board_xyz'), dict(who_am_i=0x70),
                            dict(scale=[1, 1, float('nan')]), dict(offset_g=[0, 0]),
                            dict(scale=[1, 1, -1])]:
                path.write_text(json.dumps(good | changes))
                with self.assertRaises(ValueError):
                    load(path)

    def test_capture_detection_fit_and_repeated_pose_validation(self):
        inverse_mount = mounting('-y,x,z')
        board_poses = [(0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0),
                       (0, -1, 0), (0, 1, 0), (0, 0, 1)]
        lines = []
        sequence = 0
        for board in board_poses:
            sensor = inverse_mount(board)
            raw = [(sensor[i]/self.scale[i]+self.offset[i]) for i in range(3)]
            for j in range(420):
                a = [round((v+.0001*math.sin(j))/.000061) for v in raw]
                lines.append(','.join(map(str, ['IMU', round(sequence*1000/60), sequence, 0, *a, 0, 0, 0])))
                sequence += 1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'capture.txt'
            path.write_text('\n'.join(lines))
            result = analyse(path)
        self.assertEqual(len(result['poses']), 7)
        self.assertEqual(result['poses'][4]['axis'], '-Y')
        self.assertLess(result['validation_norm_rms_g'], .001)
        for i in range(3):
            self.assertAlmostEqual(result['calibration']['offset_g'][i], self.offset[i], places=3)


if __name__ == '__main__':
    unittest.main()
