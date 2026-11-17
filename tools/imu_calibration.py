"""Six-pose accelerometer calibration; sensor-frame g in, corrected g out.

Fit an axis-aligned ellipsoid to static gravity vectors. Unlike endpoint-only
calibration, this does not require the six holds to be perfectly axis aligned.
It cannot estimate cross-axis coupling, mounting rotation, or gyro scale.
"""
import argparse
import hashlib
import json
from math import sqrt
from pathlib import Path
from statistics import mean, pstdev

from imu_model import BOARD_AXES, mounting, norm, parse


def solve(matrix, target):
    rows = [list(row) + [value] for row, value in zip(matrix, target)]
    for col in range(len(rows)):
        pivot = max(range(col, len(rows)), key=lambda i: abs(rows[i][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        if abs(divisor) < 1e-10:
            raise ValueError('Insufficient independent orientations for calibration')
        rows[col] = [v/divisor for v in rows[col]]
        for i in range(len(rows)):
            if i != col:
                factor = rows[i][col]
                rows[i] = [x-factor*y for x, y in zip(rows[i], rows[col])]
    return [row[-1] for row in rows]


def fit(vectors):
    if len(vectors) < 6:
        raise ValueError('At least six independent stationary vectors required')
    rows = [[*(x*x for x in v), *v] for v in vectors]
    matrix = [[sum(r[i]*r[j] for r in rows) for j in range(6)] for i in range(6)]
    coefficients = solve(matrix, [sum(r[i] for r in rows) for i in range(6)])
    a, d = coefficients[:3], coefficients[3:]
    if any(x <= 0 for x in a):
        raise ValueError('Measurements do not define a physical ellipsoid')
    offset = [-d[i]/(2*a[i]) for i in range(3)]
    denominator = 1 + sum(a[i]*offset[i]**2 for i in range(3))
    scale = [sqrt(x/denominator) for x in a]
    if any(abs(x) > .15 for x in offset) or any(not .85 <= x <= 1.15 for x in scale):
        raise ValueError('Implausible correction: inspect poses, units and sensor configuration')
    return dict(offset_g=offset, scale=scale)


def correct(vector, calibration):
    return tuple((vector[i]-calibration['offset_g'][i])*calibration['scale'][i] for i in range(3))


def load(path):
    result = json.loads(Path(path).read_text(encoding='utf-8'))
    if result.get('schema') != 1 or result.get('frame') != 'sensor_xyz' or result.get('who_am_i') != 0x71:
        raise ValueError('Expected schema 1 LSM6DSV16BX calibration in sensor_xyz frame')
    for key in ('offset_g', 'scale'):
        if len(result[key]) != 3:
            raise ValueError('Calibration must have three offsets and three scales')
    if any(not -.15 <= x <= .15 for x in result['offset_g']) or any(not .85 <= x <= 1.15 for x in result['scale']):
        raise ValueError('Calibration contains invalid or implausible values')
    return result


def analyse(path, axes=BOARD_AXES):
    content = Path(path).read_bytes()
    samples = [s for line in content.decode('utf-8').splitlines() if (s := parse(line))]
    if not samples:
        raise ValueError('No valid IMU samples')
    transform = mounting(axes)
    times = [((s.tick-samples[0].tick) & 0xffffffff)/1000 for s in samples]
    # Coarse pose detection; window statistics below reject transition motion.
    runs = []
    for i, s in enumerate(samples):
        v = transform(s.accel)
        axis = max(range(3), key=lambda k: abs(v[k]))
        label = ('+' if v[axis] >= 0 else '-') + 'XYZ'[axis]
        if not .75 <= norm(v) <= 1.25 or abs(v[axis])/norm(v) < .9:
            label = None
        continuous = i > 0 and 0 < times[i]-times[i-1] <= .1 and s.sequence == (samples[i-1].sequence+1) & 0xffffffff
        if runs and runs[-1][0] == label and continuous:
            runs[-1][2] = i+1
        else:
            runs.append([label, i, i+1])
    poses = []
    for label, first, end in runs:
        if label is None or times[end-1]-times[first] < 5:
            continue
        candidates = []
        for start in range(first+30, end-180, 15):
            block = samples[start:start+180]  # About three seconds at 60 Hz.
            noise = max(pstdev(s.accel[k] for s in block) for k in range(3))
            motion = max(pstdev(s.gyro[k] for s in block) for k in range(3))
            if noise <= .01 and motion <= 1.5:
                candidates.append((noise, start))
        if not candidates:
            continue
        noise, start = min(candidates)
        block = samples[start:start+180]
        average = lambda part: [mean(s.accel[k] for s in part) for k in range(3)]
        poses.append(dict(axis=label, start_s=times[start], end_s=times[start+179],
                          samples=180, max_std_g=noise, mean_sensor_g=average(block),
                          mean_board_g=transform(average(block)),
                          train_sensor_g=average(block[:90]), test_sensor_g=average(block[90:]),
                          test_vectors=[s.accel for s in block[90:]]))
    selected = []
    for label in ('+Z', '-Z', '+X', '-X', '-Y', '+Y'):
        matches = [p for p in poses if p['axis'] == label]
        if not matches:
            raise ValueError(f'No sufficiently stable {label} hold; repeat that pose')
        selected.append(matches[0])
    calibration = fit([p['train_sensor_g'] for p in selected])
    errors, raw_errors = [], []
    for pose in poses:
        corrected = correct(pose['test_sensor_g'], calibration)
        pose['corrected_mean_board_g'] = transform(corrected)
        pose['raw_norm_g'] = norm(pose['test_sensor_g'])
        pose['corrected_norm_g'] = norm(corrected)
        test_vectors = pose.pop('test_vectors')
        raw_errors.extend(norm(v)-1 for v in test_vectors)
        residuals = [norm(correct(v, calibration))-1 for v in test_vectors]
        pose['validation_norm_rms_g'] = sqrt(mean(x*x for x in residuals))
        errors.extend(residuals)
        if pose['validation_norm_rms_g'] > .01:
            raise ValueError(f"{pose['axis']} validation error exceeds 0.01 g; no calibration exported")
    calibration.update(schema=1, frame='sensor_xyz', who_am_i=0x71,
                       source_sha256=hashlib.sha256(content).hexdigest())
    report = dict(source=str(path), source_sha256=calibration['source_sha256'], axes=axes,
                  total_samples=len(samples), duration_s=times[-1], poses=poses,
                  raw_validation_norm_rms_g=sqrt(mean(x*x for x in raw_errors)),
                  calibration=calibration, validation_norm_rms_g=sqrt(mean(x*x for x in errors)))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture')
    parser.add_argument('--axes', default=BOARD_AXES)
    parser.add_argument('--output', required=True, help='Calibration JSON; loaded only when explicitly requested')
    parser.add_argument('--report', required=True, help='Analysis JSON including selected time windows')
    args = parser.parse_args()
    report = analyse(args.capture, args.axes)
    for path, value in [(args.output, report['calibration']), (args.report, report)]:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
