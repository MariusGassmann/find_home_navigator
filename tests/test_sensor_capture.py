from contextlib import redirect_stdout, redirect_stderr
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import sensor_capture


class CaptureTests(unittest.TestCase):
    def run_capture(self, directory, reads, seconds='0'):
        path = Path(directory)/'capture.txt'
        device = MagicMock()
        device.read_until.side_effect = reads
        context = MagicMock()
        context.__enter__.return_value = device
        args = ['sensor_capture', '--port', 'fake', '--seconds', seconds, '--log', str(path)]
        with patch.object(sys, 'argv', args), patch.object(sensor_capture.serial, 'Serial', return_value=context), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            sensor_capture.main()
        return path.read_bytes()

    def test_no_time_limit_stops_cleanly_and_preserves_partial_records(self):
        with tempfile.TemporaryDirectory() as directory:
            log = self.run_capture(directory, [b'RX,10,0,', b'ff\r\n', b'MAG,10,0,1,2,3,1\r\n', KeyboardInterrupt()])
        self.assertIn(b'RX,10,0,ff\r\nMAG,10,0,1,2,3,1\r\n', log)
        self.assertIn(b'# Capture end reason=ctrl_c', log)

    def test_disconnect_keeps_data_and_records_error(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(SystemExit) as error:
                self.run_capture(directory, [b'# MAG init=0\r\n', sensor_capture.serial.SerialException('unplugged')])
            self.assertEqual(error.exception.code, 1)
            log = (Path(directory)/'capture.txt').read_bytes()
        self.assertIn(b'# MAG init=0\r\n', log)
        self.assertIn(b'# Capture end reason=error', log)

    def test_existing_capture_is_preserved_before_opening_serial(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'capture.txt'; p.write_bytes(b'original measurement')
            with patch.object(sys, 'argv', ['capture','--port','fake','--log',str(p)]), \
                    patch.object(sensor_capture.serial, 'Serial') as serial, redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): sensor_capture.main()
                serial.assert_not_called()
            self.assertEqual(p.read_bytes(), b'original measurement')

    def test_invalid_durations_are_rejected_before_opening_serial(self):
        for seconds in ('-1', 'nan', 'inf'):
            with patch.object(sys, 'argv', ['capture', '--port', 'fake', '--seconds', seconds]), \
                    patch.object(sensor_capture.serial, 'Serial') as serial, redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): sensor_capture.main()
                serial.assert_not_called()


if __name__ == '__main__':
    unittest.main()
