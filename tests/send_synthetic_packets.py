#!/usr/bin/env python3
"""UDP 回环集成测试的发送端。

按 Unitree L1 的报文布局构造合法报文发给指定地址，用于在没有任何雷达的
环境下验证 udp_receiver 与 packet_parser 在真实 socket 上的行为。

decoder_selftest 覆盖的是协议、解析与解码三层，但 **udp_receiver.cpp 本身
没有离线覆盖**（它需要真的 socket）。这个脚本补上那一块：报文走一遍
sendto -> 内核 -> recvfrom -> 解析器，端到端确认收包与定界没有问题。

布局与 protocol.hpp 的 pack(1) 镜像结构体一致：
    FrameHeader  12 B   55 AA 05 0A | packet_type u32 | packet_size u32
    数据段             DataInfo 16 + InsideState 36 + CalibParam 32
                       + 线信息 36 + ranges[300] u16 + intensities[300] u8
    FrameTail    12 B   crc32 u32 | msg_type_check u32 | reserve 2 | 00 FF
CRC32 覆盖「帧头 + 数据段」，不含帧尾本身。
"""

import argparse
import socket
import struct
import sys
import time

HEADER_MARK = bytes([0x55, 0xAA, 0x05, 0x0A])
TAIL_MARK = bytes([0x00, 0xFF])

TYPE_POINT_CLOUD_3D = 102
TYPE_IMU = 104

# 与 protocol.hpp 的 kMaxPoints3D 保持一致
MAX_POINTS_3D = 300


def crc32_iso(data: bytes) -> int:
    """标准 CRC-32/ISO-HDLC：反射多项式 0xEDB88320，初值 0xFFFFFFFF，结果取反。"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if (crc & 1) else (crc >> 1)
    return (~crc) & 0xFFFFFFFF


def seal(packet_type: int, body: bytes) -> bytes:
    """补齐帧头与帧尾（含 CRC），返回完整报文。"""
    total = 12 + len(body) + 12
    head = HEADER_MARK + struct.pack("<II", packet_type, total)
    payload = head + body
    crc = crc32_iso(payload)
    tail = struct.pack("<II", crc, packet_type) + b"\x00\x00" + TAIL_MARK
    return payload + tail


def point_cloud_body(seq: int, point_num: int) -> bytes:
    """构造一个点云数据段。标定参数全置 0，几何退化为 YZ 平面极坐标，便于核对。"""
    info = struct.pack("<IIII", seq, 1020, 1000, 0)
    state = struct.pack("<II", 1, 1) + struct.pack("<7f", *([0.0] * 7))
    param = struct.pack("<8f", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001)

    # 8 个 float 线信息 + point_num
    line = struct.pack(
        "<8f",
        0.0,             # com_horizontal_angle_start
        0.0,             # com_horizontal_angle_step
        0.01,            # scan_period
        0.0,             # range_min
        100.0,           # range_max
        0.0,             # angle_min
        0.0,             # angle_increment
        0.0,             # time_increment
    )
    line += struct.pack("<I", point_num)

    # 有效点给 1000 mm（1 m），无效点给 0，用于验证剔除逻辑
    ranges = [1000] * point_num + [0] * (MAX_POINTS_3D - point_num)
    intensities = [(i % 256) for i in range(MAX_POINTS_3D)]

    return info + state + param + line + struct.pack(f"<{MAX_POINTS_3D}H", *ranges) + bytes(intensities)


def imu_body(seq: int) -> bytes:
    """构造一个 IMU 数据段。四元数第 0 分量置 1，便于核对顺序参数。"""
    info = struct.pack("<IIII", seq, 56, 2000, 500000000)
    quat = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
    angular = struct.pack("<3f", 0.1, 0.2, 0.3)
    accel = struct.pack("<3f", 0.0, 0.0, 9.8)
    return info + quat + angular + accel


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="目标地址，默认 127.0.0.1")
    parser.add_argument("--port", type=int, default=46201, help="目标端口")
    parser.add_argument("--count", type=int, default=6, help="发送多少个合法点云包")
    parser.add_argument("--corrupt", type=int, default=1, help="额外发送多少个 CRC 损坏的包")
    parser.add_argument("--imu", type=int, default=0, help="额外发送多少个 IMU 包")
    parser.add_argument("--interval", type=float, default=0.02, help="包间隔秒数")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    sent = 0
    for seq in range(args.count):
        pkt = seal(TYPE_POINT_CLOUD_3D, point_cloud_body(seq, 100))
        sock.sendto(pkt, target)
        sent += 1
        print(f"发送 点云 seq={seq} {len(pkt)} 字节")
        time.sleep(args.interval)

    for i in range(args.corrupt):
        pkt = bytearray(seal(TYPE_POINT_CLOUD_3D, point_cloud_body(900 + i, 100)))
        # 改数据段里的一个字节，CRC 随之失配，应被解析器拒收
        pkt[12 + 40] ^= 0xFF
        sock.sendto(bytes(pkt), target)
        sent += 1
        print(f"发送 CRC 损坏包 #{i} {len(pkt)} 字节")

    for seq in range(args.imu):
        pkt = seal(TYPE_IMU, imu_body(seq))
        sock.sendto(pkt, target)
        sent += 1
        print(f"发送 IMU seq={seq} {len(pkt)} 字节")

    sock.close()
    print(f"共发送 {sent} 个 UDP 包")
    return 0


if __name__ == "__main__":
    sys.exit(main())
