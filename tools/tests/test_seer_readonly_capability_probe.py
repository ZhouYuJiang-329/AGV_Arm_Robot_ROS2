"""Offline checks: status framing and refusal of all tested write APIs."""
import importlib.util
import json
from pathlib import Path
import struct
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'seer_readonly_capability_probe.py'
SPEC = importlib.util.spec_from_file_location('readonly_probe', MODULE)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class MemorySocket:
    def __init__(self, response=b''):
        self.response = response
        self.sent = []

    def settimeout(self, timeout):
        pass

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, length):
        part = self.response[:min(length, 3)]
        self.response = self.response[len(part):]
        return part


def reply(api, data, sequence=1, compressed=False):
    body = json.dumps(data).encode()
    reserved = bytearray(6)
    reserved[4] = int(compressed)
    return PROBE.HEADER.pack(0x5A, 1, sequence, len(body), api + 10000,
                             bytes(reserved)) + body


class ReadonlyProbeTest(unittest.TestCase):
    def test_write_and_motion_apis_are_rejected_before_send(self):
        for api in (2010, 3003, 3051, 3055, 3056, 3066, 4005, 4091):
            sock = MemorySocket()
            with self.assertRaises(ValueError):
                PROBE.StatusClient(sock, 1).query(api, {'vx': 0})
            self.assertEqual(sock.sent, [])

    def test_fragmented_status_response_keeps_native_header(self):
        value = {'ret_code': 0, 'motor_info': [
            {'header': {'data_nsec': '1234', 'seq': '0'}}]}
        sock = MemorySocket(reply(1040, value))
        row = PROBE.StatusClient(sock, 1).query(1040)
        self.assertEqual(row['response'], value)
        self.assertEqual(PROBE.HEADER.unpack(sock.sent[0][:16])[4], 1040)

    def test_wrong_request_sequence_is_rejected(self):
        sock = MemorySocket(reply(1005, {'ret_code': 0}, sequence=2))
        with self.assertRaises(RuntimeError):
            PROBE.StatusClient(sock, 1).query(1005)

    def test_compressed_response_is_not_treated_as_plain_json(self):
        sock = MemorySocket(reply(1005, {'ret_code': 0}, compressed=True))
        with self.assertRaises(RuntimeError):
            PROBE.StatusClient(sock, 1).query(1005)

    def test_upload_time_is_not_inventoried_as_measurement_time(self):
        candidates = PROBE.timestamp_candidates({
            'create_on': 'upload time',
            'header': {'data_nsec': '1234', 'pub_nsec': '1250'}})
        self.assertNotIn('create_on', [x['path'] for x in candidates])
        self.assertIn('header.data_nsec', [x['path'] for x in candidates])


if __name__ == '__main__':
    unittest.main()
