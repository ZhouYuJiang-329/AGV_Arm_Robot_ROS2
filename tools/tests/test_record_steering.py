#!/usr/bin/python3
"""Memory-socket proof that steering capture sends API1005 on status port only."""
import csv
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from record_steering import SteeringRecorder
from seer_readonly_capability_probe import HEADER


def exact(sock, length):
    data = b''
    while len(data) < length:
        part = sock.recv(length - len(data))
        if not part:
            return None
        data += part
    return data


class SteeringTest(unittest.TestCase):
    def test_status_wire_and_raw_angle_order_preserved(self):
        client, server = socket.socketpair()
        requests = []
        failures = []
        response = {'ret_code': 0, 'steer_angles': [.1, -.2], 'r_steer_angles': [.3, -.4],
                    'motor_steer_angles': [{'motor_name': 'rotation2', 'value': -.2}],
                    'vx': 0., 'vy': 0., 'w': 0., 'is_stop': True}
        def serve():
            try:
                while True:
                    raw = exact(server, HEADER.size)
                    if raw is None:
                        return
                    sync, version, seq, size, api, reserved = HEADER.unpack(raw)
                    requests.append((api, size))
                    if (sync, version, size, api) != (0x5A, 1, 0, 1005):
                        raise AssertionError('unexpected request')
                    body = json.dumps(response).encode()
                    server.sendall(HEADER.pack(0x5A, 1, seq, len(body), 11005, bytes(6)) + body)
            except Exception as error:
                failures.append(str(error))
            finally:
                server.close()
        worker = threading.Thread(target=serve)
        worker.start()
        def connect(address, timeout):
            self.assertEqual(address, ('127.0.0.1', 19204))
            self.assertLessEqual(timeout, .3)
            return client
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SteeringRecorder(tmp, '127.0.0.1', rate_hz=10., connector=connect)
            try:
                recorder.start()
                end = time.monotonic() + 2
                while recorder.summary()['samples'] < 3 and time.monotonic() < end:
                    time.sleep(.01)
            finally:
                recorder.stop()
                worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
            self.assertFalse(failures)
            self.assertGreaterEqual(recorder.summary()['samples'], 3)
            self.assertEqual(recorder.summary()['errors'], 0)
            self.assertTrue(all(api == 1005 and size == 0 for api, size in requests))
            rows = [json.loads(line) for line in (Path(tmp) / 'steering.jsonl').read_text().splitlines()]
            self.assertEqual(rows[0]['response']['steer_angles'], [.1, -.2])
            self.assertEqual(rows[0]['response']['r_steer_angles'], [.3, -.4])
            with (Path(tmp) / 'steering.csv').open() as stream:
                table = list(csv.DictReader(stream))
            self.assertEqual(json.loads(table[0]['actual_steer_rad_json']), [.1, -.2])
            self.assertEqual(float(table[0]['vx']), 0.)
            self.assertFalse(recorder.summary()['sample_age_verified'])

    def test_connection_failure_is_recorded_and_shutdown_is_bounded(self):
        def fail(address, timeout):
            raise ConnectionRefusedError('test disconnect')
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SteeringRecorder(tmp, '127.0.0.1', connector=fail)
            recorder.start()
            end = time.monotonic() + 1
            while recorder.summary()['errors'] == 0 and time.monotonic() < end:
                time.sleep(.01)
            start = time.monotonic()
            recorder.stop()
            self.assertLess(time.monotonic() - start, 1.)
            self.assertGreater(recorder.summary()['errors'], 0)
            self.assertEqual(recorder.summary()['samples'], 0)
            self.assertIn('test disconnect', recorder.summary()['last_error'])
            self.assertIn('test disconnect', (Path(tmp) / 'steering.jsonl').read_text())


if __name__ == '__main__':
    unittest.main()
