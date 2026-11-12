#!/usr/bin/env python3
"""Minimal WiFi ground-station for HomeFly (UDP control link).

Sends HomeFly ControlFrames (see src/communication/control_protocol.hpp) to a
running ``homefly_linux_native --wifi`` session. Frame layout (little-endian):

    offset  size  field
    0       4     magic   = 0x48464C43 ("HFLC")
    4       1     version = 1
    5       1     type    (1=Arm 2=Disarm 3=Setpoint 4=Ping 5=Mode)
    6       2     seq     (uint16)
    8       16    payload (Setpoint = 4 floats: throttle, roll, pitch, yaw_rate)
    24      4     checksum (FNV-1a over bytes [0,24), seed 2166136261)

The flight controller listens on its ``--wifi-port``; point this tool at that
host:port. Ctrl-C sends a DISARM frame and exits cleanly.

Example
-------
    # terminal A: start the flight controller in WiFi mode (mock plant)
    ./build/linux/src/applications/linux_native/homefly_linux_native \
        --steps 2000 --wifi --wifi-port 14540

    # terminal B: drive it
    python tools/wifi_gcs.py --target-port 14540 --throttle 0.5
"""

import argparse
import socket
import struct
import sys
import time

MAGIC = 0x48464C43
VERSION = 1

ARM = 1
DISARM = 2
SETPOINT = 3
PING = 4
MODE = 5

# FNV-1a 32-bit, matches control_frame_checksum() in C.
_FNV_OFFSET = 2166136261
_FNV_PRIME = 16777619


def fnv1a(data: bytes) -> int:
    h = _FNV_OFFSET
    for b in data:
        h ^= b
        h = (h * _FNV_PRIME) & 0xFFFFFFFF
    return h


def build_frame(ftype: int, payload: bytes = b"", seq: int = 0) -> bytes:
    if len(payload) > 16:
        raise ValueError("payload must be <= 16 bytes")
    body = struct.pack("<IBBH", MAGIC, VERSION, ftype, seq & 0xFFFF)
    body += payload.ljust(16, b"\x00")
    body += struct.pack("<I", fnv1a(body))
    return body


def setpoint_payload(throttle: float, roll: float, pitch: float, yaw_rate: float) -> bytes:
    return struct.pack("<ffff", throttle, roll, pitch, yaw_rate)


def main() -> int:
    parser = argparse.ArgumentParser(description="HomeFly WiFi ground-station (UDP).")
    parser.add_argument("--target-host", default="127.0.0.1",
                        help="flight controller host (default 127.0.0.1)")
    parser.add_argument("--target-port", type=int, default=14540,
                        help="flight controller --wifi-port (default 14540)")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="setpoint send rate in Hz (default 20)")
    parser.add_argument("--throttle", type=float, default=0.5,
                        help="throttle command 0..1 (default 0.5)")
    parser.add_argument("--roll", type=float, default=0.0,
                        help="roll setpoint in radians (default 0.0)")
    parser.add_argument("--pitch", type=float, default=0.0,
                        help="pitch setpoint in radians (default 0.0)")
    parser.add_argument("--yaw-rate", type=float, default=0.0,
                        help="yaw rate setpoint rad/s (default 0.0)")
    parser.add_argument("--count", type=int, default=0,
                        help="stop after N setpoint frames (0 = until Ctrl-C)")
    parser.add_argument("--rearm-every", type=int, default=0, dest="rearm_every",
                        help="re-send DISARM+ARM every N setpoints (pilot re-arm "
                             "gesture). The only way to recover a stick Failsafe "
                             "while the link is alive (0 = disabled)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target_host, args.target_port)
    seq = 0

    def send(ftype: int, payload: bytes = b"") -> None:
        nonlocal seq
        sock.sendto(build_frame(ftype, payload, seq), target)
        seq += 1

    print(f"[wifi_gcs] target {args.target_host}:{args.target_port}, rate={args.rate} Hz")
    print("[wifi_gcs] sending ARM ...")
    send(ARM)

    interval = 1.0 / args.rate if args.rate > 0 else 0.0
    sent = 0
    next_time = time.monotonic()
    try:
        while True:
            payload = setpoint_payload(args.throttle, args.roll, args.pitch, args.yaw_rate)
            if args.rearm_every and sent > 0 and sent % args.rearm_every == 0:
                print("[wifi_gcs] re-arming (DISARM+ARM) ...")
                send(DISARM)
                # Let the receiver consume DISARM in its own control cycle: the
                # FSM needs a DisarmRequest step to leave Failsafe before the
                # ARM request can re-enter Stabilize. Two back-to-back frames
                # may be drained by a single poll() and cancel each other out.
                time.sleep(0.15)
                send(ARM)
            send(SETPOINT, payload)
            sent += 1
            if args.count and sent >= args.count:
                break
            next_time += interval
            sleep_for = next_time - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\n[wifi_gcs] Ctrl-C: sending DISARM")
    finally:
        send(DISARM)
        sock.close()
        print(f"[wifi_gcs] sent {sent} setpoint frame(s), disarmed. bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
