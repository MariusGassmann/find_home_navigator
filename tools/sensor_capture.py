"""Capture GNSS/compass diagnostics, preserving RX hex and raw MAG records."""
import argparse
import math
from datetime import datetime, timezone
from pathlib import Path
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=180, help='Duration; 0 runs until Ctrl+C')
    parser.add_argument('--baud', type=int, default=460800)
    parser.add_argument('--log', type=Path, default=Path('captures/gnss.txt'))
    parser.add_argument('--overwrite', action='store_true', help='Explicitly replace an existing capture')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative (0 = until Ctrl+C)')
    if args.log.exists() and not args.overwrite:
        parser.error('Log already exists; choose a new filename or use --overwrite')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    try:
        with serial.Serial(args.port, args.baud, timeout=.2) as port, args.log.open('wb' if args.overwrite else 'xb') as log:
            port.reset_input_buffer()
            header = f'# Host capture UTC={datetime.now(timezone.utc).isoformat()} port={args.port} baud={args.baud}\n'
            log.write(header.encode())
            print(header, end='')
            print('Press shield RESET now to capture startup, IDs and first UART bytes. Ctrl+C stops.')
            started = time.monotonic()
            end = started + args.seconds if args.seconds else math.inf
            reason = 'duration'
            print(f'Time limit: {args.seconds:g} s' if args.seconds else 'No time limit; stop with Ctrl+C.')
            try:
                while time.monotonic() < end:
                    data = port.read_until(b'\n', 1024)
                    if data:
                        log.write(data)
                        log.flush()
                        if not data.startswith((b'RX,', b'MAG,', b'IMU,', b'FUSE,')):
                            print(data.decode('ascii', errors='replace'), end='', flush=True)
            except KeyboardInterrupt:
                reason = 'ctrl_c'
            except (serial.SerialException, OSError):
                reason = 'error'
                raise
            finally:
                footer = f'\n# Capture end reason={reason} elapsed_host_s={time.monotonic()-started:.1f}\n'
                log.write(footer.encode())
    except (serial.SerialException, OSError) as error:
        parser.exit(1, f'Capture failed: {error}\n')
    print(f'\nSaved {args.log}')


if __name__ == '__main__':
    main()
