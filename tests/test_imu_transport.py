import queue
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from imu_model import Attitude, parse
from imu_viewer import Session, serial_reader


class SerialCommandTests(unittest.TestCase):
    def test_partial_reads_and_commands_share_one_serial_connection(self):
        stop = threading.Event()
        lines, commands = queue.Queue(), queue.Queue()
        commands.put(b'r'); commands.put(b't')
        written = []
        chunks = iter([b'IM', b'', b'U,17,1,0,0,0,16393,0,0,0\r\n# hello', b'\r\n'])

        class Device:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def write(self, command): written.append(command)
            def read_until(self, *args):
                try:
                    return next(chunks)
                except StopIteration:
                    stop.set()
                    return b''

        fake = SimpleNamespace(Serial=lambda *a, **k: Device(), SerialException=RuntimeError)
        with patch.dict(sys.modules, {'serial': fake}):
            serial_reader('fake', lines, commands, stop)
        self.assertEqual(written, [b'r', b't'])
        self.assertIsNotNone(parse(lines.get_nowait()))
        self.assertEqual(lines.get_nowait(), '# hello\r')
        self.assertTrue(lines.empty())

    def test_open_error_is_reported(self):
        def unavailable(*args, **kwargs):
            raise OSError('port busy')
        fake = SimpleNamespace(Serial=unavailable, SerialException=RuntimeError)
        lines = queue.Queue()
        with patch.dict(sys.modules, {'serial': fake}):
            serial_reader('fake', lines, queue.Queue(), threading.Event())
        self.assertIn('# ERROR serial: port busy', lines.get_nowait())

    def test_reference_live_and_demo_and_pixel_test(self):
        for port in ('COM3', None):
            session = Session.__new__(Session)
            session.args = SimpleNamespace(port=port)
            session.log = None
            session.model = Attitude()
            session.commands = queue.Queue()
            session.reference()
            self.assertTrue(session.model.calibrating)
            session.matrix_test()
            if port:
                self.assertEqual(session.commands.get_nowait(), b'r')
                self.assertEqual(session.commands.get_nowait(), b't')
            self.assertTrue(session.commands.empty())


if __name__ == '__main__':
    unittest.main()
