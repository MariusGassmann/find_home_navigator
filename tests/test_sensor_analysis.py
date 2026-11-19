from pathlib import Path
import math
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from sensor_analysis import Segment, gga_position, local_points, read_capture, summarize


def nmea(body):
    data = body.encode()
    checksum = 0
    for byte in data: checksum ^= byte
    return b'$'+data+f'*{checksum:02X}\r\n'.encode()


FIX = 'GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'


class RawAnalysisTests(unittest.TestCase):
    def test_chunk_boundaries_and_checksum_failure(self):
        raw = nmea(FIX)
        for split in range(len(raw)+1):
            s = Segment()
            s.raw(10, 0, raw[:split]); s.raw(20, split, raw[split:])
            self.assertEqual(s.counts['nmea_valid'], 1)
            self.assertEqual(len(s.positions), 1)
            self.assertAlmostEqual(s.positions[0][1], 48.1173)
        s = Segment()
        s.raw(0, 0, raw.replace(b'123519', b'123518'))
        self.assertEqual(s.counts['nmea_checksum_errors'], 1)
        self.assertFalse(s.positions)

    def test_mixed_binary_and_nmea(self):
        # UBX payload contains '$' and newline: must not be mistaken for NMEA.
        payload = b'$\n\x00\xff'
        body = b'\x01\x07'+len(payload).to_bytes(2, 'little')+payload
        a = b = 0
        for byte in body:
            a = (a+byte) & 255; b = (b+a) & 255
        raw = b'noise'+b'\xb5\x62'+body+bytes((a, b))+nmea(FIX)
        s = Segment()
        for i, byte in enumerate(raw): s.raw(i, i, bytes((byte,)))
        self.assertEqual(s.counts['ubx_01_07'], 1)
        self.assertEqual(s.counts['noise_bytes'], 5)
        self.assertEqual(s.counts['nmea_valid'], 1)
        self.assertEqual(len(s.positions), 1)

    def test_missing_chunks_cannot_join_a_sentence(self):
        raw = nmea(FIX)
        s = Segment(); s.raw(0, 0, raw[:20]); s.raw(1, 30, raw[30:])
        self.assertEqual(s.counts['capture_offset_gaps'], 1)
        self.assertFalse(s.positions)
        s.raw(2, len(raw), raw)
        self.assertEqual(len(s.positions), 1)

    def test_long_incomplete_and_malformed_frames(self):
        s = Segment()
        s.raw(0, 0, b'$'+b'A'*600+b'\n'+b'$broken*ZZ\n'+nmea('GNGGA,0,invalid'))
        self.assertEqual(s.counts['nmea_overlong'], 1)
        self.assertEqual(s.counts['nmea_framing_errors'], 1)
        self.assertEqual(s.counts['gga_field_errors'], 1)
        self.assertFalse(s.positions)

    def test_coordinate_bounds_modes_and_nan(self):
        self.assertIsNone(gga_position('GNGGA,0,,,,,0,0,,,,', 0))
        for body in (FIX.replace('4807.038', '4860.0'), FIX.replace(',N,', ',E,'),
                     FIX.replace('0.9', 'nan'), FIX.replace('01131.000', '18100.000')):
            with self.assertRaises(ValueError): gga_position(body, 0)
        for quality in (0, 3, 6, 7, 8):
            self.assertIsNone(gga_position(FIX.replace(',1,08,', f',{quality},08,'), 0))
        position = gga_position(FIX.replace(',N,', ',S,').replace(',E,', ',W,'), 0)
        self.assertLess(position[1], 0); self.assertLess(position[2], 0)

    def test_projection_metres_and_dateline(self):
        positions = [(0, 0, 0, 8, 1), (1, .001, .001, 8, 1)]
        _, points = local_points(positions)
        self.assertAlmostEqual(points[1][0]-points[0][0], 111.1949, places=3)
        self.assertAlmostEqual(points[1][1]-points[0][1], 111.1949, places=3)
        _, points = local_points([(0, 0, 179.999, 8, 1), (1, 0, -179.999, 8, 1)])
        self.assertLess(max(math.hypot(*p) for p in points), 300)

    def test_capture_resets_loss_and_legacy_fallback(self):
        raw = nmea(FIX)
        text = ('# SENSOR v2\nRX,20,0,'+raw.hex()+'\nMAG,20,0,1500,0,0,1\n'
                'MAG,40,1,30000,0,0,3\nGNSS,1000,FIX,lat_e7=481173000,lon_e7=115166667,sat=8,hdop_x100=90\n'
                '# RX_LOSS errors=1 dropped=0\nRX,9999,4,broken# SENSOR v2\n# MAG init=0\n'
                'GNSS,1000,FIX,lat_e7=10000000,lon_e7=20000000,sat=9,hdop_x100=100\n'
                'GNSS,10,NO_FIX,lat_e7=10000000,lon_e7=20000000,sat=0,hdop_x100=0\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'capture.txt'; path.write_text(text)
            segments = read_capture(path)
        self.assertEqual(len(segments), 3)
        first = summarize(segments[0])
        self.assertEqual(first['position_source'], 'raw GGA')
        self.assertEqual(first['magnetic_usable'], 1)
        self.assertEqual(first['magnetic_overflow_records'], 1)
        self.assertEqual(first['magnetic_norm_uT_range'], [40, 40])
        self.assertEqual(first['counts']['receiver_loss_events'], 1)
        self.assertEqual(summarize(segments[1])['position_source'], 'status (1 Hz)')
        self.assertEqual(summarize(segments[1])['compass_initialization'], '# MAG init=0')
        self.assertEqual(summarize(segments[2])['position_samples'], 0)


if __name__ == '__main__':
    unittest.main()
