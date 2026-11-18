#!/usr/bin/env python3
"""Read-only inventory of Seer sample headers and native script availability.

Only status port 19204 and an explicit status-API allowlist are used. This tool
never sends a motion/zero, navigation, configuration, or script-execution request.
It does not certify freshness and never changes commissioning flags.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import sys
import time

HEADER = struct.Struct('>BBHIH6s')
STATUS_PORT = 19204
ALLOWED_APIS = frozenset((1000, 1002, 1004, 1005, 1040, 1506))
MAX_BODY = 4 * 1024 * 1024


def receive_exact(sock, length, deadline):
    parts = bytearray()
    while len(parts) < length:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('status request deadline exceeded')
        sock.settimeout(remaining)
        part = sock.recv(length - len(parts))
        if not part:
            raise RuntimeError('controller closed status connection')
        parts.extend(part)
    return bytes(parts)


class StatusClient:
    def __init__(self, sock, timeout):
        self.sock = sock
        self.timeout = timeout
        self.sequence = 0

    def query(self, api, payload=None):
        if api not in ALLOWED_APIS:
            raise ValueError('API is not in the read-only allowlist')
        self.sequence += 1
        body = b'' if payload is None else json.dumps(payload).encode('utf-8')
        sent = time.monotonic()
        deadline = sent + self.timeout
        self.sock.settimeout(self.timeout)
        self.sock.sendall(HEADER.pack(
            0x5A, 1, self.sequence, len(body), api, bytes(6)) + body)
        sync, version, sequence, size, kind, reserved = HEADER.unpack(
            receive_exact(self.sock, HEADER.size, deadline))
        if (sync, version, sequence, kind) != (
                0x5A, 1, self.sequence, api + 10000):
            raise RuntimeError('status response header mismatch')
        if size > MAX_BODY or reserved[4] != 0:
            raise RuntimeError('oversize or compressed response unsupported')
        response = json.loads(receive_exact(self.sock, size, deadline)) if size else {}
        if not isinstance(response, dict):
            raise RuntimeError('status response must be a JSON object')
        return {'api': api, 'request': payload, 'sent_monotonic_s': sent,
                'received_monotonic_s': time.monotonic(), 'response': response}


def timestamp_candidates(value, path=''):
    """Inventory field names; a matching name does not establish semantics."""
    found = []
    if isinstance(value, dict):
        for key, item in value.items():
            name = path + '.' + key if path else key
            if key in ('header', 'data_nsec', 'pub_nsec', 'seq', 'timestamp',
                       'sample_time', 'sample_timestamp', 'received_on'):
                found.append({'path': name, 'value': item})
            found.extend(timestamp_candidates(item, name))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            found.extend(timestamp_candidates(item, path + '[' + str(index) + ']'))
    return found


def collect(client, samples, interval):
    report = {'read_only': True, 'status_port': STATUS_PORT,
              'motion_sent': False, 'freshness_verified': False,
              'interpretation': 'Candidate fields and clock comparisons only; '
              'create_on is API upload time, not accepted as measurement time.',
              'inventory': [], 'clock_samples': []}
    info = client.query(1000)
    report['controller'] = {k: info['response'].get(k) for k in (
        'version', 'current_map', 'current_map_md5', 'model_md5', 'calibrated')}
    for api in (1004, 1005):
        row = client.query(api)
        response = row['response']
        report['inventory'].append({
            'api': api, 'ret_code': response.get('ret_code', 0),
            'fields': sorted(response),
            'timestamp_candidates': timestamp_candidates(response)})
    scripts = client.query(1506, {'type': 'genetic'})
    report['script_inventory'] = scripts['response']
    for _ in range(samples):
        before = client.query(1002)
        motors = client.query(1040)
        after = client.query(1002)
        sample = {'before': before, 'motors': motors, 'after': after,
                  'candidate_ages_if_same_clock': []}
        for motor in motors['response'].get('motor_info', []):
            stamp = motor.get('header', {}).get('data_nsec')
            if stamp is None:
                continue
            try:
                data_seconds = int(stamp) / 1e9
                lo = float(before['response']['time']) / 1000 - data_seconds
                hi = float(after['response']['time']) / 1000 - data_seconds
            except (ValueError, TypeError, KeyError):
                continue
            sample['candidate_ages_if_same_clock'].append({
                'motor': motor.get('motor_name'), 'header': motor.get('header'),
                'age_interval_s': [lo, hi],
                'warning': 'Assumes data_nsec is ns in the same clock epoch '
                'as API1002 uptime; this assumption must be validated.'})
        report['clock_samples'].append(sample)
        time.sleep(interval)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--interval', type=float, default=0.3)
    parser.add_argument('--timeout', type=float, default=2.0)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100:
        parser.error('--samples must be 1..100')
    if not math.isfinite(args.interval) or not 0.1 <= args.interval <= 10:
        parser.error('--interval must be finite and 0.1..10 seconds')
    if not math.isfinite(args.timeout) or not 0.1 <= args.timeout <= 10:
        parser.error('--timeout must be finite and 0.1..10 seconds')
    if args.output.exists():
        parser.error('output exists; choose a new filename')
    try:
        with socket.create_connection((args.ip, STATUS_PORT), timeout=args.timeout) as sock:
            report = collect(StatusClient(sock, args.timeout), args.samples, args.interval)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        print('Read-only report saved:', args.output)
        for row in report['inventory']:
            print('API', row['api'], 'candidate fields:',
                  [v['path'] for v in row['timestamp_candidates']])
        print('Freshness has NOT been certified; no execution flags changed.')
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print('Read-only probe failed:', exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
