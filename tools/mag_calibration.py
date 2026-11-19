"""Fit and independently validate QMC5883P hard/soft-iron calibration.

Model: corrected_uT = matrix @ (raw_uT - offset_uT), in sensor axes.
Symmetric positive-definite correction follows NXP AN4246; determinant one
preserves geometric mean scale. Absolute field strength and mounting rotation
cannot be established from free rotations. No samples are silently trimmed.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from sensor_analysis import read_capture


def vectors(values):
    v = np.asarray(values, dtype=float)
    if v.ndim != 2 or v.shape[1] != 3 or len(v) < 300 or not np.isfinite(v).all():
        raise ValueError('Need at least 300 finite XYZ samples')
    return v


def correct(values, calibration):
    return (np.asarray(values) - calibration['offset_uT']) @ np.asarray(calibration['matrix']).T


def coverage(values):
    lengths = np.linalg.norm(values, axis=1)
    if np.any(lengths < 1e-6):
        raise ValueError('Zero field vector; inspect data')
    directions = values / lengths[:, None]
    eigen = np.linalg.eigvalsh(directions.T @ directions / len(directions))
    # Require broad support on both sides of all axes, plus 3D directional spread.
    low, high = directions.min(axis=0), directions.max(axis=0)
    good = eigen[0] >= .08 and np.all(low < -.65) and np.all(high > .65)
    return dict(passed=bool(good), direction_eigenvalues=eigen.tolist(),
                direction_min=low.tolist(), direction_max=high.tolist())


def metrics(raw, calibration):
    corrected = correct(raw, calibration)
    radius = np.linalg.norm(corrected, axis=1)
    error = np.abs(radius/calibration['field_uT'] - 1)
    return dict(samples=len(raw), coverage=coverage(corrected),
                raw_norm_range_uT=[float(x) for x in (np.linalg.norm(raw, axis=1).min(), np.linalg.norm(raw, axis=1).max())],
                corrected_norm_range_uT=[float(radius.min()), float(radius.max())],
                rms_fraction=float(np.sqrt(np.mean(error**2))),
                p95_fraction=float(np.quantile(error, .95)), max_fraction=float(error.max()))


def acceptable(report):
    return (report['coverage']['passed'] and report['rms_fraction'] <= .05
            and report['p95_fraction'] <= .10 and report['max_fraction'] <= .20)


def _ellipsoid(values):
    v = vectors(values)
    origin = v.mean(axis=0)
    scale = float(np.sqrt(np.mean(np.sum((v-origin)**2, axis=1))))
    if scale < 1:
        raise ValueError('Insufficient field variation; rotate around all three axes')
    x, y, z = ((v-origin)/scale).T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    c, _, rank, singular = np.linalg.lstsq(design, np.ones(len(v)), rcond=None)
    if rank < 9 or singular[0]/singular[-1] > 1000:
        raise ValueError('Insufficient independent orientations; rotate all three axes')
    q = np.array([[c[0], c[3], c[4]], [c[3], c[1], c[5]], [c[4], c[5], c[2]]])
    if np.linalg.eigvalsh(q).min() <= 0:
        raise ValueError('Data does not describe a physical ellipsoid')
    centre = -.5*np.linalg.solve(q, c[6:])
    eigen, basis = np.linalg.eigh(q/(1 + centre @ q @ centre))
    root = np.sqrt(eigen)
    common = np.prod(root)**(1/3)
    gains = root/common
    if gains.min() < .25 or gains.max() > 4:
        raise ValueError('Implausible distortion; inspect mounting and magnetic interference')
    cal = dict(schema=1, sensor='QMC5883P', frame='sensor_xyz', range_gauss=8,
               counts_per_uT=37.5, offset_uT=(origin+scale*centre).tolist(),
               matrix=((basis*gains) @ basis.T).tolist(), field_uT=float(scale/common))
    if not 15 <= cal['field_uT'] <= 100:
        raise ValueError('Implausible fitted field scale; inspect units and interference')
    return cal


def fit(values):
    v = vectors(values)
    cal = _ellipsoid(v)
    report = metrics(v, cal)
    if not acceptable(report):
        raise ValueError('Training data failed coverage/field checks: ' + json.dumps(report))
    return cal


def load(path):
    c = json.loads(Path(path).read_text(encoding='utf-8'))
    if (c.get('schema'), c.get('sensor'), c.get('frame'), c.get('range_gauss'), c.get('counts_per_uT')) != (1, 'QMC5883P', 'sensor_xyz', 8, 37.5):
        raise ValueError('Expected QMC5883P +/-8 G sensor-frame calibration, schema 1')
    offset, matrix = np.asarray(c.get('offset_uT'), dtype=float), np.asarray(c.get('matrix'), dtype=float)
    if offset.shape != (3,) or matrix.shape != (3,3) or not np.isfinite(offset).all() or not np.isfinite(matrix).all():
        raise ValueError('Invalid offset or matrix')
    if not np.allclose(matrix, matrix.T, atol=1e-8):
        raise ValueError('Correction must be symmetric; mounting rotation is separate')
    eigen = np.linalg.eigvalsh(matrix)
    if eigen.min() < .25 or eigen.max() > 4 or not np.isclose(np.linalg.det(matrix), 1, atol=1e-5):
        raise ValueError('Invalid correction gains')
    if not 15 <= c.get('field_uT', 0) <= 100:
        raise ValueError('Invalid reference field')
    return c


def capture(path):
    segments = read_capture(path)
    if not segments:
        raise ValueError('No sensor capture found')
    content = Path(path).read_bytes()
    boots = sum('# SENSOR ' in line or '# GNSS discovery:' in line
                for line in content.decode('utf-8', errors='replace').splitlines())
    # Capture starts before the instructed RESET; discard only that pre-reset prefix.
    if boots > 1 or (len(segments) > 1 and not (boots == 1 and len(segments) == 2)):
        raise ValueError('Capture contains multiple board boots; record one uninterrupted session')
    rows = segments[-1].magnetic
    usable = [r for r in rows if r[5] & 1 and not r[5] & 2]
    if any(r[5] & 2 for r in rows):
        raise ValueError('Magnetic overflow; move away from strong magnetic sources')
    if len(usable) < 300:
        raise ValueError('Need at least 300 usable MAG samples')
    if ((usable[-1][0]-usable[0][0]) & 0xffffffff) < 20000:
        raise ValueError('Record at least 20 seconds of slow rotations')
    v = vectors([[r[i]/37.5 for i in (2,3,4)] for r in usable])
    return v, dict(path=str(path), sha256=hashlib.sha256(content).hexdigest(),
                   discarded_pre_reset_segments=len(segments)-1, samples=len(v), sequence_gaps=sum(b[1] != ((a[1]+1) & 0xffffffff) for a,b in zip(usable, usable[1:])))


def plot(training, validation, cal, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(11,7), layout='constrained')
    for row in range(2):
        for ax, (i,j) in zip(axes[row], ((0,1),(0,2),(1,2))):
            for values, label in ((training,'fit'), (validation,'validation')):
                v = values if row == 0 else correct(values, cal)
                ax.scatter(v[:,i], v[:,j], s=2, alpha=.4, label=label)
            ax.set(xlabel='XYZ'[i]+' [uT]', ylabel='XYZ'[j]+' [uT]', aspect='equal',
                   title='Raw' if row == 0 else 'Corrected')
            ax.grid(alpha=.3)
    axes[0,0].legend()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path, help='Training capture')
    p.add_argument('--validation', type=Path, required=True, help='Separate rotations, same mounting/environment')
    p.add_argument('--output', type=Path, default=Path('calibration/mag.json'))
    p.add_argument('--report', type=Path, default=Path('captures/mag-calibration'))
    p.add_argument('--no-plots', action='store_true')
    a = p.parse_args()
    paths = [a.capture.resolve(), a.validation.resolve(), a.output.resolve()]
    if len(set(paths)) != 3 or a.output.exists():
        p.error('Use distinct inputs/output and a new output filename; existing calibration is preserved')
    a.report.mkdir(parents=True, exist_ok=True)
    report = dict(accepted=False)
    try:
        train, source = capture(a.capture)
        check, validation_source = capture(a.validation)
        if np.array_equal(train, check):
            raise ValueError('Validation repeats the training data')
        cal = _ellipsoid(train)
        report.update(training=metrics(train, cal), validation=metrics(check, cal),
                      source=source, validation_source=validation_source)
        if not a.no_plots:
            plot(train, check, cal, a.report/'comparison.png')
        if not acceptable(report['training']):
            raise ValueError('Training failed coverage or field checks; see report and comparison plot')
        if not acceptable(report['validation']):
            raise ValueError('Independent validation failed coverage or field checks; see report')
        cal.update(source=source, validation_source=validation_source,
                   quality=report, method='symmetric ellipsoid; determinant-one scale')
        a.output.parent.mkdir(parents=True, exist_ok=True)
        # Publish only after both datasets pass. Exclusive creation preserves old files.
        report['accepted'] = True
        with a.output.open('x', encoding='utf-8') as f:
            json.dump(cal, f, indent=2, allow_nan=False)
            f.write('\n')
        print(f'Saved calibration: {a.output}')
    except (ValueError, OSError, np.linalg.LinAlgError) as error:
        report['accepted'] = False
        report['error'] = str(error)
    (a.report/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
