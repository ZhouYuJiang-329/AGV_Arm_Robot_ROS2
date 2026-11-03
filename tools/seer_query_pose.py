#!/usr/bin/env python3
"""Learn the Seer TCP API by sending one read-only 1004 position query."""

import argparse
import json
import socket
import struct
import sys


# Big endian: sync, version, sequence, body length, type, six reserved bytes.
HEADER = struct.Struct('>BBHIH6s')


def receive_exact(sock, length):
    result = bytearray()
    while len(result) < length:
        part = sock.recv(length - len(result))
        if not part:
            raise RuntimeError('连接提前关闭，响应报文未接收完整')
        result.extend(part)
    return bytes(result)


def query_pose(host, port=19204, version=1, timeout=5):
    sequence = 1
    # API 1004 has no request body: length is zero.
    request = HEADER.pack(0x5A, version, sequence, 0, 1004, b'\0' * 6)
    print('发送报文：', ' '.join(f'{byte:02X}' for byte in request))
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(request)
        sync, reply_version, reply_sequence, length, reply_type, reserved = HEADER.unpack(
            receive_exact(sock, HEADER.size))
        if (sync, reply_version, reply_sequence, reply_type) != (0x5A, version, sequence, 11004):
            raise RuntimeError('响应头不匹配：同步头、版本、序号或响应编号与请求不符')
        if length > 4 * 1024 * 1024:
            raise RuntimeError('响应体长度异常，停止接收')
        if reserved[4] != 0:
            raise RuntimeError('收到压缩响应，本入门例子仅支持未压缩 JSON')
        raw = receive_exact(sock, length)
    data = json.loads(raw.decode('utf-8')) if raw else {}
    if not isinstance(data, dict):
        raise RuntimeError('响应 JSON 应为对象')
    print('返回 JSON：\n' + json.dumps(data, ensure_ascii=False, indent=2))
    if data.get('ret_code', 0) != 0:
        raise RuntimeError(f'机器人返回错误：{data.get("ret_code")} {data.get("err_msg", "")}')
    for field in ('x', 'y', 'angle'):
        if field not in data:
            raise RuntimeError(f'响应中缺少 {field}，请检查定位状态和 API 版本')
    print(f'位置：x={data["x"]} m，y={data["y"]} m，朝向={data["angle"]} rad')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True, help='机器人控制器实际 IP')
    parser.add_argument('--port', type=int, default=19204)
    parser.add_argument('--version', type=int, choices=(1, 2), default=1,
                        help='协议头版本：RBK3.4 用 1，RBK3.5 用 2；默认 1')
    parser.add_argument('--timeout', type=float, default=5)
    args = parser.parse_args()
    try:
        query_pose(args.ip, args.port, args.version, args.timeout)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'查询失败：{exc}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
