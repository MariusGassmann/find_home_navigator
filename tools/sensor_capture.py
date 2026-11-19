"""Capture the standalone GNSS/compass discovery firmware (USB COM, 115200)."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--log', type=Path, default=Path('captures/gnss.txt'))
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    try:
        with serial.Serial(args.port, 115200, timeout=.2) as port, args.log.open('wb') as log:
            header = f'# Host capture UTC={datetime.now(timezone.utc).isoformat()} port={args.port}\n'
            log.write(header.encode())
            print(header, end='')
            print('Press shield RESET now to capture startup, IDs and first UART bytes. Ctrl+C stops.')
            end = time.monotonic() + args.seconds
            try:
                while time.monotonic() < end:
                    data = port.read_until(b'\n', 1024)
                    if data:
                        log.write(data)
                        log.flush()
                        print(data.decode('ascii', errors='replace'), end='', flush=True)
            except KeyboardInterrupt:
                pass
    except (serial.SerialException, OSError) as error:
        parser.exit(1, f'Capture failed: {error}\n')
    print(f'\nSaved {args.log}')


if __name__ == '__main__':
    main()
