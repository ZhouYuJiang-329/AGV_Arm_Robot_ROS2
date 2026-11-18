#!/usr/bin/python3
"""Bounded read-only API1005 sampler; never connects to a control port."""
import csv
import json
from pathlib import Path
import socket
import threading
import time

from seer_readonly_capability_probe import StatusClient, STATUS_PORT


class SteeringRecorder:
    def __init__(self, output, ip, rate_hz=5., connector=socket.create_connection):
        self.output = Path(output)
        self.ip = ip
        self.period = 1. / rate_hz
        self.connector = connector
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.state = {'enabled': True, 'ip': ip, 'status_port': STATUS_PORT,
                      'api': 1005, 'requested_rate_hz': rate_hz, 'samples': 0, 'errors': 0,
                      'last_error': None, 'max_query_ms': 0., 'sample_age_verified': False,
                      'actual_angle_order': 'controller model order; not assumed front/rear'}
        self.thread = threading.Thread(target=self.run, name='readonly_steering_sampler', daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.done.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('read-only steering sampler did not finish within 2 seconds')

    def summary(self):
        with self.lock:
            return dict(self.state)

    def run(self):
        fields = ['request_unix_sec', 'receive_unix_sec', 'query_ms', 'api_upload_time',
                  'actual_steer_rad_json', 'received_steer_rad_json', 'motor_steer_raw_json',
                  'vx', 'vy', 'w', 'r_vx', 'r_vy', 'r_w', 'is_stop', 'error']
        with (self.output / 'steering.jsonl').open('x') as raw, (
                self.output / 'steering.csv').open('x', newline='') as table:
            writer = csv.DictWriter(table, fieldnames=fields)
            writer.writeheader()
            while not self.done.is_set():
                try:
                    # Fixed status port only. Separate connection from the navigation driver.
                    with self.connector((self.ip, STATUS_PORT), timeout=.3) as sock:
                        client = StatusClient(sock, .3)
                        while not self.done.is_set():
                            cycle = time.monotonic()
                            request_wall = time.time()
                            reply = client.query(1005)
                            received_wall = time.time()
                            response = reply['response']
                            if response.get('ret_code', 0) != 0:
                                raise RuntimeError('API1005 returned nonzero ret_code')
                            elapsed = (reply['received_monotonic_s'] - reply['sent_monotonic_s']) * 1000
                            raw.write(json.dumps({'request_unix_sec': request_wall,
                                'receive_unix_sec': received_wall, **reply}, ensure_ascii=False) + '\n')
                            row = {'request_unix_sec': request_wall, 'receive_unix_sec': received_wall,
                                   'query_ms': elapsed, 'api_upload_time': response.get('create_on'),
                                   'actual_steer_rad_json': json.dumps(response.get('steer_angles')),
                                   'received_steer_rad_json': json.dumps(response.get('r_steer_angles')),
                                   'motor_steer_raw_json': json.dumps(response.get('motor_steer_angles'))}
                            row.update({k: response.get(k) for k in ('vx', 'vy', 'w', 'r_vx', 'r_vy', 'r_w', 'is_stop')})
                            writer.writerow(row)
                            raw.flush()
                            table.flush()
                            with self.lock:
                                self.state['samples'] += 1
                                self.state['last_error'] = None
                                self.state['last_receive_unix_sec'] = received_wall
                                self.state['max_query_ms'] = max(self.state['max_query_ms'], elapsed)
                                self.state['last_actual_steer_rad'] = response.get('steer_angles')
                                self.state['last_received_steer_rad'] = response.get('r_steer_angles')
                                self.state['last_motor_steer_raw'] = response.get('motor_steer_angles')
                            if self.done.wait(max(0., self.period - (time.monotonic() - cycle))):
                                break
                except Exception as error:
                    message = type(error).__name__ + ': ' + str(error)
                    with self.lock:
                        self.state['errors'] += 1
                        self.state['last_error'] = message
                    raw.write(json.dumps({'receive_unix_sec': time.time(), 'error': message}) + '\n')
                    writer.writerow({'receive_unix_sec': time.time(), 'error': message})
                    raw.flush()
                    table.flush()
                    self.done.wait(.5)
