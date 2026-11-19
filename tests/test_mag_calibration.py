import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from mag_calibration import fit, correct, metrics, acceptable, load, capture, main
from compass_view import Compass


def synthetic(seed=10, n=1000):
    rng = np.random.default_rng(seed)
    direction = rng.normal(size=(n,3))
    direction /= np.linalg.norm(direction, axis=1)[:,None]
    distortion = np.array([[1.2,.12,-.08],[.12,.85,.04],[-.08,.04,1.04]])
    distortion /= np.linalg.det(distortion)**(1/3)
    offset = np.array([16.,-12.,24.])
    truth = direction*52
    raw = truth @ distortion.T + offset + rng.normal(0,.06,(n,3))
    return raw, truth


def write_capture(path, values, status=25):
    path.write_text('\n'.join(f'MAG,{i*40},{i},' + ','.join(str(round(x*37.5)) for x in v)
                              + f',{status}' for i,v in enumerate(values))+'\n')


class CalibrationTests(unittest.TestCase):
    def test_recovers_offset_full_matrix_and_direction(self):
        for seed in range(5):
            raw, _ = synthetic(seed)
            cal = fit(raw)
            np.testing.assert_allclose(cal['offset_uT'], [16,-12,24], atol=.05)
            self.assertAlmostEqual(np.linalg.det(cal['matrix']), 1, places=8)
            validation, truth = synthetic(seed+100)
            result = correct(validation, cal)
            self.assertLess(np.sqrt(np.mean((result-truth)**2)), .1)
            cosine = np.sum(result*truth, axis=1)/(np.linalg.norm(result, axis=1)*52)
            self.assertLess(np.quantile(np.degrees(np.arccos(np.clip(cosine,-1,1))), .99), .5)
            self.assertTrue(acceptable(metrics(validation, cal)))

    def test_bad_coverage_and_nonfinite_data_rejected(self):
        angle = np.linspace(0,2*np.pi,1000)
        planar = np.column_stack((50*np.cos(angle), 50*np.sin(angle), np.ones(1000)*20))
        for raw in (planar, np.ones((1000,3)), np.ones((20,3)),
                    np.full((1000,3),np.nan), np.ones((1000,2))):
            with self.assertRaises(ValueError): fit(raw)
        raw, _ = synthetic()
        with self.assertRaises(ValueError): fit(raw[:300]*[1,1,.001])

    def test_validation_detects_interference_and_missing_orientations(self):
        raw, _ = synthetic()
        cal = fit(raw)
        check, truth = synthetic(200)
        self.assertFalse(acceptable(metrics(check+[30,0,0], cal)))
        self.assertFalse(acceptable(metrics(check[truth[:,2]>25], cal)))
        self.assertFalse(acceptable(metrics(check*1.3, cal)))
        check[::10] += [70,0,0]
        self.assertFalse(acceptable(metrics(check, cal)))

    def test_export_load_viewer_and_input_safety(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            train, validation = d/'fit.txt', d/'validate.txt'
            output, report = d/'mag.json', d/'report'
            write_capture(train, synthetic()[0])
            write_capture(validation, synthetic(20)[0])
            argv = ['mag_calibration', str(train), '--validation', str(validation),
                    '--output', str(output), '--report', str(report), '--no-plots']
            with patch.object(sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(), 0)
            cal = load(output)
            self.assertTrue(cal['quality']['accepted'])
            c = Compass(cal)
            c.update('MAG,10,0,1000,2000,3000,25')
            np.testing.assert_allclose(c.points[-1], correct([[1000/37.5,2000/37.5,3000/37.5]], cal)[0])
            before = output.read_bytes()
            with patch.object(sys, 'argv', argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): main()
            self.assertEqual(output.read_bytes(), before)
            # A rejected validation never publishes a usable calibration file.
            output.unlink()
            write_capture(validation, synthetic(20)[0]+[35,0,0])
            with patch.object(sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(), 1)
            self.assertFalse(output.exists())
            self.assertFalse(json.loads((report/'report.json').read_text())['accepted'])
            # A rejected fit still reports both datasets for diagnosis; never publish it.
            write_capture(train, synthetic()[0]+np.random.default_rng(1).normal(0,10,(1000,3)))
            with patch.object(sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(), 1)
            rejected = json.loads((report/'report.json').read_text())
            self.assertIn('training', rejected)
            self.assertIn('validation', rejected)
            self.assertFalse(output.exists())

    def test_loader_rejects_rotation_nan_wrong_sensor_and_gain(self):
        base = fit(synthetic()[0])
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'cal.json'
            for key,value in [('matrix', [[0,-1,0],[1,0,0],[0,0,1]]),
                              ('matrix', np.eye(3).tolist()), ('sensor','QMC5883L'),
                              ('offset_uT',[float('nan'),0,0]), ('field_uT',float('inf'))]:
                c = dict(base); c[key] = value
                if key == 'matrix' and value == np.eye(3).tolist():
                    c[key] = (np.eye(3)*2).tolist()
                p.write_text(json.dumps(c))
                with self.assertRaises(ValueError): load(p)

    def test_capture_rejects_overflow_short_and_multiple_boots(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'capture.txt'
            values = synthetic()[0]
            write_capture(p, values, status=3)
            with self.assertRaises(ValueError): capture(p)
            write_capture(p, values[:200])
            with self.assertRaises(ValueError): capture(p)
            write_capture(p, values)
            p.write_text('# SENSOR first boot\n'+p.read_text()+'# SENSOR reboot\nMAG,0,0,1,2,3,25\n')
            with self.assertRaises(ValueError): capture(p)

    def test_requested_reset_discards_prefix_without_mixing_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'capture.txt'
            values = synthetic()[0]
            write_capture(p, values)
            records = p.read_bytes()
            p.write_bytes(b'MAG,90000,999,30000,30000,30000,3\nRX,90001,0,ab\xff# SENSOR v5\n' + records)
            actual, source = capture(p)
            self.assertEqual(len(actual), len(values))
            self.assertEqual(source['discarded_pre_reset_segments'], 1)
            np.testing.assert_allclose(actual, values, atol=1/37.5)
