"""Run the hardware command chain against four local SEER mock ports only."""
import json
import socket
import struct
import subprocess
import sys
import threading


def exact(conn, size):
    data = b""
    while len(data) < size:
        part = conn.recv(size - len(data))
        if not part:
            if not data:
                return None
            raise RuntimeError("partial frame")
        data += part
    return data


def main():
    sockets = []
    threads = []
    requests = []
    errors = []
    lock = threading.Lock()

    def serve(listener, port):
        try:
            conn, _ = listener.accept()
            with conn:
                conn.settimeout(10)
                while True:
                    header = exact(conn, 16)
                    if header is None:
                        return
                    magic, version, sequence, length, api = struct.unpack(">BBHIH", header[:10])
                    assert (magic, version) == (0x5A, 1)
                    assert header[10:] == bytes(6)
                    body = exact(conn, length) if length else b""
                    payload = json.loads(body) if body else None
                    with lock:
                        requests.append((port, api, payload))
                        count = len(requests)
                    reply = json.dumps({"ret_code": 42 if count == 2 else 0}).encode()
                    conn.sendall(struct.pack(">BBHIH6x", 0x5A, 1, sequence, len(reply), api + 10000) + reply)
        except Exception as exc:
            with lock:
                errors.append(str(exc))

    try:
        # Bind before starting the child. Never connect to the real robot.
        for port in (19204, 19205, 19206, 19210):
            listener = socket.socket()
            sockets.append(listener)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", port))
            listener.listen(1)
            listener.settimeout(10)
            thread = threading.Thread(target=serve, args=(listener, port), daemon=True)
            threads.append(thread)
            thread.start()
        result = subprocess.run([sys.argv[1], "--real"], capture_output=True, text=True, timeout=20)
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        assert result.returncode == 0, "hardware test failed"
        for thread in threads:
            thread.join(2)
        assert not any(thread.is_alive() for thread in threads), "connections not released"
        assert not errors, errors
        assert len(requests) == 3, requests
        assert all(port == 19205 and api == 2010 for port, api, _ in requests), requests
        assert requests[0][2] == {"vx": .1, "vy": .2, "w": -.3, "duration": 800}, requests
        assert requests[1][2] == {"vx": .2, "vy": 0, "w": .1, "duration": 800}, requests
        assert requests[2][2] == requests[0][2], requests
        print("PASS: hardware JSON -> AGV -> Client -> Transport; error response; cleanup")
    finally:
        for listener in sockets:
            listener.close()


if __name__ == "__main__":
    main()
