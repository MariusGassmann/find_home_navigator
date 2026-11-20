"""Generate firmware constants from the measured JSON files (standard library only)."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def generate(accel_path, mag_path, output):
    accel_bytes, mag_bytes = accel_path.read_bytes(), mag_path.read_bytes()
    a, m = json.loads(accel_bytes), json.loads(mag_bytes)
    if (a['schema'] != 1 or a['frame'] != 'sensor_xyz' or a['who_am_i'] != 0x71 or
            m['schema'] != 1 or m['frame'] != 'sensor_xyz' or m['sensor'] != 'QMC5883P' or
            m['range_gauss'] != 8 or m['counts_per_uT'] != 37.5 or not m['quality']['accepted']):
        raise ValueError('Calibration does not match the configured sensors')

    def number(value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('Non-finite calibration')
        return repr(value) + 'f'

    def vector(values):
        if len(values) != 3:
            raise ValueError('Expected three axes')
        return '{' + ', '.join(map(number, values)) + '}'

    if any(v <= 0 for v in a['scale']) or m['field_uT'] <= 0 or len(m['matrix']) != 3:
        raise ValueError('Invalid scale, field or matrix')
    fields = [vector(a['offset_g']), vector(a['scale']), vector(m['offset_uT']),
              '{' + ', '.join(map(vector, m['matrix'])) + '}', number(m['field_uT'])]
    fingerprint = hashlib.sha256(accel_bytes + mag_bytes).hexdigest()[:16]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('/* Generated from calibration JSON files; do not edit. */\n'
                      '#include "north.h"\n'
                      f'#define NORTH_CALIBRATION_ID "{fingerprint}"\n'
                      'static const north_calibration_t north_calibration = {\n  ' +
                      ',\n  '.join(fields) + '\n};\n', encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('accel', type=Path)
    parser.add_argument('mag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    generate(args.accel, args.mag, args.output)
