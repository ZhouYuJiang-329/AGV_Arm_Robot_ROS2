#!/usr/bin/env python3
"""Read-only 1005 baseline then concurrent 1005/1009 probe; no control APIs."""
import argparse
import concurrent.futures
import json
import math
import socket
import struct
import time
from pathlib import Path

HEADER = struct.Struct('>BBHIH6s')


def collect(host, api, seconds, rate):
    rows = []
    end = time.monotonic() + seconds
    sock = None
    sequence = 0
    while time.monotonic() < end:
        start = time.monotonic()
        row = {'start': start, 'api': api}
        try:
            if sock is None:
                sock = socket.create_connection((host, 19204), timeout=1)
            deadline = time.monotonic() + 1

            def exact(count):
                data = b''
                while len(data) < count:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('transaction deadline exceeded')
                    sock.settimeout(remaining)
                    part = sock.recv(count - len(data))
                    if not part:
                        raise RuntimeError('EOF')
                    data += part
                return data

            sequence = (sequence + 1) % 65536
            sock.sendall(HEADER.pack(0x5a, 1, sequence, 0, api, bytes(6)))
            sync, version, seq, size, kind, reserved = HEADER.unpack(exact(16))
            if (sync, version, seq, kind) != (0x5a, 1, sequence, api + 10000) or size > 4194304 or reserved[4]:
                raise RuntimeError('invalid frame')
            body = json.loads(exact(size))
            if body.get('ret_code', 0) != 0:
                raise RuntimeError('API error')
            if api == 1005:
                if not all(type(body.get(k)) in (int, float) and math.isfinite(body[k]) for k in ('vx', 'vy', 'w')):
                    raise RuntimeError('missing/invalid actual velocity')
                row['velocity'] = {k: body[k] for k in ('vx', 'vy', 'w')}
                row['keys'] = sorted(body)
            row['ok'] = True
        except (OSError, ValueError, RuntimeError) as exc:
            row.update(ok=False, error=str(exc))
            if sock:
                sock.close()
            sock = None
        row['latency_ms'] = (time.monotonic() - start) * 1000
        rows.append(row)
        time.sleep(max(0, min(end - time.monotonic(), 1 / rate - (time.monotonic() - start))))
    if sock:
        sock.close()
    return rows


def summary(rows, duration):
    good = [r['latency_ms'] for r in rows if r['ok']]
    ordered = sorted(good)
    return {'responses': len(good), 'failures': len(rows) - len(good),
            'valid_response_hz': len(good) / duration,
            'latency_ms': {str(q): ordered[min(len(ordered)-1, int((len(ordered)-1)*q/100))] if ordered else None
                           for q in (0, 50, 95, 99, 100)},
            'unique_device_sample_hz': 'unknown: no verified sampling timestamp'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.seconds < 60:
        parser.error('each phase must last at least 60 seconds')
    baseline = collect(args.ip, 1005, args.seconds, 5)
    print('baseline', json.dumps(summary(baseline, args.seconds)), flush=True)
    with concurrent.futures.ThreadPoolExecutor(2) as pool:
        speed = pool.submit(collect, args.ip, 1005, args.seconds, 5)
        laser = pool.submit(collect, args.ip, 1009, args.seconds, 5)
        speed, laser = speed.result(), laser.result()
    result = {'duration_per_phase_sec': args.seconds, 'request_rate_hz': 5,
              'baseline': summary(baseline, args.seconds), 'concurrent_speed': summary(speed, args.seconds),
              'concurrent_laser': summary(laser, args.seconds),
              'raw': {'baseline': baseline, 'speed': speed, 'laser': laser}}
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != 'raw'}, indent=2))


if __name__ == '__main__':
    main()
