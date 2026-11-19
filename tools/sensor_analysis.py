"""Analyze a sensor capture and export GNSS wander / raw magnetic-field plots."""
import argparse
from collections import Counter
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import statistics


def gga_position(body, tick):
    f = body.split(',')
    if len(f[0]) != 5 or not f[0][:2].isalpha() or f[0][2:] != 'GGA':
        return None
    if len(f) < 10:
        raise ValueError('short GGA')
    quality = int(f[6])
    if quality not in (1, 2, 4, 5):
        return None

    def coordinate(text, hemisphere, latitude):
        width = 2 if latitude else 3
        if not re.fullmatch(r'\d{' + str(width+2) + r'}(?:\.\d+)?', text):
            raise ValueError('coordinate syntax')
        degrees, minutes = int(text[:width]), float(text[width:])
        limit = 90 if latitude else 180
        if minutes >= 60 or degrees > limit or (degrees == limit and minutes):
            raise ValueError('coordinate bounds')
        if hemisphere not in (('N', 'S') if latitude else ('E', 'W')):
            raise ValueError('hemisphere')
        return (degrees + minutes/60) * (-1 if hemisphere in ('S', 'W') else 1)

    lat = coordinate(f[2], f[3], True)
    lon = coordinate(f[4], f[5], False)
    hdop, satellites = float(f[8]), int(f[7])
    if not math.isfinite(hdop) or not 0 < hdop <= 999.99 or not 0 <= satellites <= 99:
        raise ValueError('quality fields')
    return (tick, lat, lon, satellites, hdop)


@dataclass
class Segment:
    positions: list = field(default_factory=list)
    status_positions: list = field(default_factory=list)
    magnetic: list = field(default_factory=list)
    status: list = field(default_factory=list)
    counts: Counter = field(default_factory=Counter)
    buffer: bytearray = field(default_factory=bytearray)
    offset: int = 0
    last_tick: int | None = None
    mag_init: str | None = None
    capture_end: str | None = None

    def raw(self, tick, offset, data):
        if offset != self.offset:
            self.counts['capture_offset_gaps'] += 1
            self.buffer.clear()
        self.offset = offset + len(data)
        self.counts['raw_bytes'] += len(data)
        self.buffer.extend(data)
        while self.buffer:
            b = self.buffer
            if b[0] == ord('$'):
                end = b.find(b'\n')
                if end < 0:
                    if len(b) <= 512:
                        break
                    self.counts['nmea_overlong'] += 1
                    del b[0]
                    continue
                if end > 512:
                    self.counts['nmea_overlong'] += 1
                    del b[:end+1]
                    continue
                frame = bytes(b[1:end]).rstrip(b'\r')
                del b[:end+1]
                if len(frame)+1 > 159:
                    self.counts['nmea_exceeds_firmware_buffer'] += 1
                try:
                    body, checksum = frame.rsplit(b'*', 1)
                    expected = 0
                    for byte in body:
                        expected ^= byte
                    if len(checksum) != 2 or not re.fullmatch(b'[0-9a-fA-F]{2}', checksum):
                        raise ValueError('checksum syntax')
                    if int(checksum, 16) != expected:
                        self.counts['nmea_checksum_errors'] += 1
                        continue
                    text = body.decode('ascii')
                except (ValueError, UnicodeError):
                    self.counts['nmea_framing_errors'] += 1
                    continue
                self.counts['nmea_valid'] += 1
                try:
                    position = gga_position(text, tick)
                except (ValueError, OverflowError):
                    self.counts['gga_field_errors'] += 1
                else:
                    if position:
                        self.positions.append(position)
            elif b[0] == 0xb5:
                if len(b) < 2:
                    break
                if b[1] != 0x62:
                    del b[0]; self.counts['noise_bytes'] += 1
                    continue
                if len(b) < 6:
                    break
                size = b[4] + 256*b[5]
                if size > 4096:
                    del b[0]; self.counts['ubx_oversize'] += 1
                    continue
                if len(b) < size+8:
                    break
                a = c = 0
                for byte in b[2:size+6]:
                    a = (a+byte) & 255; c = (c+a) & 255
                if (a, c) == (b[size+6], b[size+7]):
                    self.counts[f'ubx_{b[2]:02x}_{b[3]:02x}'] += 1
                else:
                    self.counts['ubx_checksum_errors'] += 1
                del b[:size+8]
            else:
                del b[0]; self.counts['noise_bytes'] += 1


def read_capture(path):
    segments = [Segment()]
    for line in Path(path).read_text(encoding='utf-8', errors='replace').splitlines():
        s = segments[-1]
        if '# SENSOR ' in line or '# GNSS discovery:' in line:
            if s.status or s.counts or s.magnetic:
                segments.append(Segment())
            continue
        if line.startswith('# MAG init='):
            s.mag_init = line
        if line.startswith('# Capture end '):
            s.capture_end = line
        if line.startswith('# RX_LOSS'):
            s.buffer.clear(); s.counts['receiver_loss_events'] += 1
        elif line.startswith('RX,'):
            try:
                _, tick, offset, hex_data = line.split(',')
                if len(hex_data) > 256:
                    raise ValueError('oversize RX record')
                data = bytes.fromhex(hex_data)
                s.raw(int(tick), int(offset), data)
            except ValueError:
                s.counts['malformed_rx_records'] += 1
        elif line.startswith('MAG,'):
            try:
                values = [int(v) for v in line.split(',')[1:]]
                if len(values) != 6 or any(not -32768 <= v <= 32767 for v in values[2:5]):
                    raise ValueError('MAG record')
                s.magnetic.append(tuple(values))
            except ValueError:
                s.counts['malformed_mag_records'] += 1
        else:
            match = re.search(r'GNSS,(\d+),(FIX|NO_FIX|NO_DATA|STALE),(.+)', line)
            if not match:
                continue
            tick = int(match[1])
            if s.last_tick is not None and 0 < s.last_tick-tick < 0x80000000:
                segments.append(Segment()); s = segments[-1]
            s.last_tick = tick
            fields = {k: int(v) for k, v in re.findall(r'(\w+)=(-?\d+)', match[3])}
            s.status.append((tick, match[2], fields))
            if match[2] == 'FIX' and all(k in fields for k in ('lat_e7', 'lon_e7', 'sat', 'hdop_x100')):
                s.status_positions.append((tick, fields['lat_e7']/1e7, fields['lon_e7']/1e7,
                                           fields['sat'], fields['hdop_x100']/100))
    return [s for s in segments if s.status or s.counts or s.magnetic]


def local_points(positions):
    lat0 = statistics.median(p[1] for p in positions)
    reference = positions[0][2]
    lon0 = statistics.median(reference + (p[2]-reference+180) % 360-180 for p in positions)
    lon0 = (lon0+180) % 360-180
    points = [(6371000*math.radians((p[2]-lon0+180) % 360-180)*math.cos(math.radians(lat0)),
               6371000*math.radians(p[1]-lat0)) for p in positions]
    return (lat0, lon0), points


def summarize(s):
    positions = s.positions or s.status_positions
    result = {'counts': dict(s.counts), 'position_source': 'raw GGA' if s.positions else 'status (1 Hz)',
              'position_samples': len(positions), 'magnetic_samples': len(s.magnetic),
              'raw_incomplete_tail_bytes': len(s.buffer),
              'note': 'Position spread is not absolute accuracy. Motion also contributes.'}
    if s.status:
        result['last_status'] = {'state': s.status[-1][1], **s.status[-1][2]}
        result['status_counts'] = dict(Counter(row[1] for row in s.status))
    if s.mag_init:
        result['compass_initialization'] = s.mag_init
    if s.capture_end:
        result['capture_end'] = s.capture_end
    if not s.counts['raw_bytes']:
        result['reception_note'] = 'No raw UART records: rejected sentences cannot be diagnosed from this log.'
    elif any(key.startswith('ubx_') and re.fullmatch(r'ubx_[0-9a-f]{2}_[0-9a-f]{2}', key)
             for key in s.counts):
        result['reception_note'] = 'Binary UBX present; the firmware NMEA-only parser may count binary data as bad lines.'
    if positions:
        center, points = local_points(positions)
        radii = sorted(math.hypot(x, y) for x, y in points)
        result.update(center_lat_lon=center, radius_median_m=statistics.median(radii),
                      radius_95_m=radii[math.ceil(.95*len(radii))-1],
                      satellites_range=[min(p[3] for p in positions), max(p[3] for p in positions)],
                      hdop_range=[min(p[4] for p in positions), max(p[4] for p in positions)])
    usable = [m for m in s.magnetic if m[5] & 1 and not m[5] & 2]
    result['magnetic_usable'] = len(usable)
    result['magnetic_overflow_records'] = sum(bool(m[5] & 2) for m in s.magnetic)
    if usable:
        norms = [math.sqrt(sum(v*v for v in m[2:5]))/37.5 for m in usable]
        result['magnetic_norm_uT_range'] = [min(norms), max(norms)]
    return result


def plots(s, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    positions = s.positions or s.status_positions
    if positions or s.status:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout='constrained')
        origin = s.status[0][0] if s.status else positions[0][0]
        # Raw GGA may precede the first 1 Hz status by up to a second.
        if positions and positions[0][0] < origin:
            origin = positions[0][0]
        seconds = [((p[0]-origin) & 0xffffffff)/1000 for p in positions]
        if positions:
            _, points = local_points(positions)
            scatter = axes[0].scatter([p[0] for p in points], [p[1] for p in points], c=seconds, s=9)
            fig.colorbar(scatter, ax=axes[0], label='Elapsed [s]')
        else:
            axes[0].text(.5, .5, 'No valid position samples', ha='center', transform=axes[0].transAxes)
        axes[0].set(xlabel='East of median [m]', ylabel='North of median [m]',
                    title='Position spread (includes any movement)', aspect='equal')
        axes[0].set_box_aspect(1)
        diagnostic = [(row[0], row[2].get('sat', 0), row[2].get('hdop_x100', 0)/100)
                      for row in s.status] or [(p[0], p[3], p[4]) for p in positions]
        diagnostic_seconds = [((row[0]-origin) & 0xffffffff)/1000 for row in diagnostic]
        axes[1].plot(diagnostic_seconds, [row[2] if row[2] > 0 else math.nan for row in diagnostic], label='HDOP')
        axes[1].plot(diagnostic_seconds, [row[1] for row in diagnostic], label='Satellites used')
        shaded = False
        for i, row in enumerate(s.status[:-1]):
            if row[1] != 'FIX':
                axes[1].axvspan(diagnostic_seconds[i], diagnostic_seconds[i+1], color='grey', alpha=.15,
                               label=None if shaded else 'No usable fix')
                shaded = True
        axes[1].set(xlabel='Elapsed [s]', title='Fix diagnostics'); axes[1].legend()
        for ax in axes: ax.grid(alpha=.3)
        fig.savefig(output/'gnss.png', dpi=160); plt.close(fig)
    usable = [m for m in s.magnetic if m[5] & 1 and not m[5] & 2]
    if usable:
        xyz = [[m[i]/37.5 for m in usable] for i in (2, 3, 4)]
        fig, axes = plt.subplots(2, 2, figsize=(9, 7), layout='constrained')
        for ax, (i, j) in zip(axes.flat, ((0, 1), (0, 2), (1, 2))):
            ax.scatter(xyz[i], xyz[j], s=4)
            ax.set(xlabel=f'{"XYZ"[i]} [uT]', ylabel=f'{"XYZ"[j]} [uT]', aspect='equal')
        seconds = [((m[0]-usable[0][0]) & 0xffffffff)/1000 for m in usable]
        axes[1, 1].plot(seconds, [math.sqrt(sum(m[i]**2 for i in (2, 3, 4)))/37.5 for m in usable])
        axes[1, 1].set(xlabel='Elapsed [s]', ylabel='Field magnitude [uT]')
        for ax in axes.flat: ax.grid(alpha=.3)
        fig.suptitle('Uncalibrated magnetometer: sensor axes, mounting unverified')
        fig.savefig(output/'magnetic.png', dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, default=Path('captures/analysis'))
    parser.add_argument('--segment', type=int, default=-1, help='Boot segment index; default: last')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    segments = read_capture(args.capture)
    if not segments or not -len(segments) <= args.segment < len(segments):
        parser.error('No such capture segment')
    s = segments[args.segment]
    report = summarize(s)
    report.update(source=str(args.capture), boot_segments=len(segments), selected_segment=args.segment)
    args.output.mkdir(parents=True, exist_ok=True)
    text = json.dumps(report, indent=2, allow_nan=False)
    (args.output/'summary.json').write_text(text+'\n', encoding='utf-8')
    print(text)
    if not args.no_plots:
        plots(s, args.output)
    print(f'Saved analysis to {args.output}')


if __name__ == '__main__':
    main()
