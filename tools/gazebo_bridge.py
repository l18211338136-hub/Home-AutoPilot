#!/usr/bin/env python3
"""HomeFly <-> Gazebo SITL bridge.

HomeFly never links against Gazebo. This process translates Gazebo IMU
samples into the 64-byte HomeFly wire frame and translates motor frames back
into Gazebo actuator commands. Pure standard library for the `mock` backend,
which reproduces SimPlant's rigid body attitude model so the whole SITL path
can be verified on a machine without Gazebo installed.

Protocol (little endian, 64 bytes), see simulation/gazebo/gazebo_protocol.hpp:

    struct.Struct("<IIQIBBH8fII")
    magic "HFGZ" | seq | timestamp_us | checksum | version | type | flags
    | values[8] | reserved | tail

    checksum = FNV-1a over all 64 bytes except [16:20)

Usage:
    python tools/gazebo_bridge.py --backend mock --steps 2000
    python tools/gazebo_bridge.py --backend gz --imu-topic /world/.../imu
    python tools/gazebo_bridge.py --backend classic
    python tools/gazebo_bridge.py --self-test
"""

from __future__ import annotations

import argparse
import importlib
import math
import select
import socket
import struct
import sys
import time
from typing import Callable, Optional, Sequence

FRAME = struct.Struct("<IIQIBBH8fII")
MAGIC = 0x48465A47  # "HFGZ"
VERSION = 1
TYPE_IMU = 1
TYPE_MOTOR = 2
GRAVITY = 9.80665
MOTOR_COUNT = 4

assert FRAME.size == 64, "wire frame must be exactly 64 bytes"


# --------------------------------------------------------------------------
# Wire protocol
# --------------------------------------------------------------------------

def checksum(data: bytes) -> int:
    """FNV-1a over the frame, skipping the checksum field at [16:20)."""
    value = 2166136261
    for index, byte in enumerate(data):
        if 16 <= index < 20:
            continue
        value ^= byte
        value = (value * 16777619) & 0xFFFFFFFF
    return value


def seal(data: bytes) -> bytes:
    result = bytearray(data)
    struct.pack_into("<I", result, 16, checksum(result))
    return bytes(result)


def make_imu(seq: int, timestamp_us: int, gyro: Sequence[float],
             accel: Sequence[float], temperature: float = 25.0) -> bytes:
    values = list(gyro[:3]) + list(accel[:3]) + [temperature, 0.0]
    raw = FRAME.pack(MAGIC, seq, timestamp_us, 0, VERSION, TYPE_IMU, 0,
                     *values, 0, 0)
    return seal(raw)


def make_motor(seq: int, timestamp_us: int, motors: Sequence[float]) -> bytes:
    values = list(motors[:MOTOR_COUNT]) + [0.0] * (8 - MOTOR_COUNT)
    raw = FRAME.pack(MAGIC, seq, timestamp_us, 0, VERSION, TYPE_MOTOR, 0,
                     *values, 0, 0)
    return seal(raw)


def parse_frame(data: bytes) -> Optional[dict]:
    if len(data) != FRAME.size:
        return None
    fields = FRAME.unpack(data)
    stored = fields[3]
    if fields[0] != MAGIC or fields[4] != VERSION or stored != checksum(data):
        return None
    return {
        "seq": fields[1],
        "timestamp_us": fields[2],
        "version": fields[4],
        "type": fields[5],
        "flags": fields[6],
        "values": list(fields[7:15]),
        "reserved": fields[15],
        "tail": fields[16],
    }


def self_test() -> int:
    frame = make_imu(7, 123456, (0.1, -0.2, 0.3), (-1.0, 2.0, GRAVITY))
    parsed = parse_frame(frame)
    assert parsed is not None and parsed["seq"] == 7
    assert parsed["timestamp_us"] == 123456 and parsed["type"] == TYPE_IMU
    assert abs(parsed["values"][5] - GRAVITY) < 1e-5

    motor = make_motor(3, 999, (0.25, 0.5, 0.75, 1.0))
    parsed_motor = parse_frame(motor)
    assert parsed_motor is not None and parsed_motor["type"] == TYPE_MOTOR
    assert parsed_motor["values"][3] == 1.0

    corrupted = bytearray(frame)
    corrupted[30] ^= 0x01
    assert parse_frame(bytes(corrupted)) is None, "checksum must reject tampering"

    print("gazebo_bridge self-test: PASS (frame=64B, seal/parse/CRC OK)")
    return 0


# --------------------------------------------------------------------------
# Mock plant: deterministic replica of simulation/sim_plant.cpp
# --------------------------------------------------------------------------

class MockPlant:
    """Rigid body attitude model, no aerodynamics / altitude / latency."""

    def __init__(self, roll: float, pitch: float, gyro_bias: float = 0.0):
        self.attitude = [roll, pitch, 0.0]
        self.rate = [0.0, 0.0, 0.0]
        self.motors = [0.0] * MOTOR_COUNT
        self.gyro_bias = gyro_bias

    def apply(self, motors: Sequence[float]) -> None:
        self.motors = [min(1.0, max(0.0, float(v))) for v in motors[:MOTOR_COUNT]]

    def step(self, dt_s: float) -> None:
        m = self.motors
        torques = (
            (m[1] + m[3] - m[0] - m[2]) * 0.25,
            (m[0] + m[1] - m[2] - m[3]) * 0.25,
            (m[0] + m[3] - m[1] - m[2]) * 0.25,
        )
        for axis in range(3):
            self.rate[axis] += (torques[axis] * 8.0 - self.rate[axis] * 2.5) * dt_s
            self.attitude[axis] += self.rate[axis] * dt_s

    def imu(self):
        roll, pitch, _ = self.attitude
        gyro = [self.rate[0] + self.gyro_bias, self.rate[1], self.rate[2]]
        accel = [
            -GRAVITY * math.sin(pitch),
            GRAVITY * math.sin(roll),
            GRAVITY * math.cos(roll) * math.cos(pitch),
        ]
        return gyro, accel


def make_socket(args: argparse.Namespace) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.motor_port))  # receives HomeFly motor frames
    sock.setblocking(False)
    # Windows reports WSAECONNRESET on recvfrom after sending to a port that
    # is not listening yet (the bridge normally starts before HomeFly). That
    # is harmless for SITL, but it would spin the startup retry loop.
    if sys.platform == "win32":
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, 0)
        except (AttributeError, OSError):
            pass
    return sock


def drain_rx(sock: socket.socket) -> None:
    """Discard stale datagrams before starting a new lock-step round."""
    while True:
        try:
            sock.recv(65536)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            return


def wait_motor(sock: socket.socket, timeout_s: float) -> Optional[list]:
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        ready, _, _ = select.select([sock], [], [], remaining)
        if not ready:
            return None
        try:
            data, _ = sock.recvfrom(65536)
        except (BlockingIOError, InterruptedError):
            continue
        except OSError:
            return None
        frame = parse_frame(data)
        if frame and frame["type"] == TYPE_MOTOR:
            return frame["values"][:MOTOR_COUNT]


def run_mock(args: argparse.Namespace) -> int:
    """Lock-step SITL: IMU -> HomeFly step -> motor -> plant step -> IMU."""
    sock = make_socket(args)
    target = (args.host, args.imu_port)
    plant = MockPlant(args.initial_roll, args.initial_pitch, args.gyro_bias)
    dt_s = args.period_us * 1.0e-6

    print(f"[mock] bridge ready: imu -> {target}, motors <- :{args.motor_port}, "
          f"steps={args.steps}, period={args.period_us}us")
    print("[mock] start HomeFly with: --gazebo "
          f"--gazebo-local-port {args.imu_port} "
          f"--gazebo-remote-port {args.motor_port} --steps {args.steps}")

    step = 1
    retransmits = 0
    seen_any = False
    last_progress = time.monotonic()

    while step <= args.steps:
        # Discard only STALE datagrams before each round, never the response
        # to the IMU frame we are about to send.
        drain_rx(sock)
        timestamp_us = step * args.period_us
        gyro, accel = plant.imu()
        sock.sendto(make_imu(step, timestamp_us, gyro, accel), target)

        # Short polling while waiting for HomeFly to come up, then a long
        # window: host scheduling can delay a control cycle, and needless
        # retransmits would just fill the receive queue.
        motors = wait_motor(sock, 0.05 if not seen_any else 1.0)
        if motors is None:
            retransmits += 1
            if time.monotonic() - last_progress > args.timeout:
                print(f"[mock] ERROR: no motor frame for {args.timeout}s "
                      f"(HomeFly not started or wrong ports)", file=sys.stderr)
                return 1
            # Retransmit the SAME seq. HomeFly deduplicates by seq. Throttle
            # the startup loop so waiting for HomeFly does not busy spin.
            if not seen_any:
                time.sleep(0.01)
            continue

        seen_any = True
        last_progress = time.monotonic()
        plant.apply(motors)
        plant.step(dt_s)
        step += 1

    sock.close()
    roll_deg = math.degrees(plant.attitude[0])
    pitch_deg = math.degrees(plant.attitude[1])
    print(f"[mock] done: {args.steps} IMU frames, retransmits={retransmits}, "
          f"final roll={roll_deg:7.3f} pitch={pitch_deg:7.3f} deg")
    return 0


# --------------------------------------------------------------------------
# Gazebo Garden / Harmonic (gz-transport-python) and Gazebo Classic (pygazebo)
# --------------------------------------------------------------------------

def import_gz(flavor: str):
    """Resolve versioned Python bindings; give an actionable error if absent."""
    if flavor == "gz":
        transport_modules = ("gz.transport13", "gz.transport12",
                             "gz.transport11", "gz.transport")
        msgs_modules = ("gz.msgs10", "gz.msgs9", "gz.msgs8")
        hint = ("pip install gz-transport-python gz-msgs-python  (or the "
                "bindings shipped with your Gazebo distribution)")
    else:
        transport_modules = ("pygazebo",)
        msgs_modules = ("gazebo.msgs",)
        hint = "pip install pygazebo protobuf   (Gazebo Classic 11)"

    transport = None
    for name in transport_modules:
        try:
            transport = importlib.import_module(name)
            break
        except ImportError:
            continue

    msgs = None
    for name in msgs_modules:
        try:
            msgs = importlib.import_module(name)
            break
        except ImportError:
            continue

    if transport is None or msgs is None:
        raise RuntimeError(
            f"backend '{flavor}' needs Python bindings that are not installed.\n"
            f"  {hint}\n"
            f"  Use --backend mock to verify the HomeFly side without Gazebo.")
    return transport, msgs


def axis_remap(vector: Sequence[float], order: Sequence[int],
               signs: Sequence[float]) -> list:
    return [float(vector[order[i]]) * signs[i] for i in range(3)]


def run_gz(args: argparse.Namespace, flavor: str) -> int:
    """Event driven bridge: Gazebo IMU topic -> UDP, UDP motors -> actuator topic.

    NOTE: this backend requires a real Gazebo installation and its versioned
    Python bindings. It is intentionally kept free of compile-time Gazebo
    dependencies on the HomeFly side. Validate topic names / message fields
    against your Gazebo release before the first flight.
    """
    transport, msgs = import_gz(flavor)
    sock = make_socket(args)
    target = (args.host, args.imu_port)
    order = [int(v) for v in args.axis_map.split(",")]
    signs = [float(v) for v in args.axis_sign.split(",")]
    if len(order) != 3 or len(signs) != 3 or sorted(order) != [0, 1, 2]:
        raise ValueError("--axis-map must be a permutation of 0,1,2")

    imu_msg = None
    actuator_msg = None
    if flavor == "gz":
        imu_mod = importlib.import_module(msgs.__name__ + ".imu_pb2")
        act_mod = importlib.import_module(msgs.__name__ + ".actuators_pb2")
        imu_msg, actuator_msg = imu_mod.IMU, act_mod.Actuators
        node = transport.Node()
    else:
        imu_msg = msgs.IMU
        actuator_msg = msgs.Actuators
        node = None  # pygazebo uses a Manager, wired below

    state = {"seq": 0, "frames": 0, "motors": 0}

    # gz-transport Python (gz.transport13 etc.) has NO Node.publish(): a
    # Publisher must be advertised first, then Publisher.publish(msg) is used.
    # Advertise once up front - doing it per motor frame would leak publishers.
    actuator_pub = None
    if node is not None:
        try:
            actuator_pub = node.advertise(args.motor_topic, actuator_msg)
        except TypeError:
            # Older bindings take the protobuf type name instead of the class.
            actuator_pub = node.advertise(
                args.motor_topic, actuator_msg.DESCRIPTOR.full_name)
        if actuator_pub is None:
            raise RuntimeError(f"cannot advertise {args.motor_topic}")

    def publish_actuators(values: Sequence[float]) -> None:
        msg = actuator_msg()
        # gz-sim-multicopter-motor-model-system reads Actuators.velocity
        # (rad/s), indexed by <motorNumber>. Tune max_rot_velocity to match
        # your SDF plugin.
        for value in values[:MOTOR_COUNT]:
            msg.velocity.append(max(0.0, float(value)) * args.max_rot_velocity)
        if actuator_pub is not None:
            actuator_pub.publish(msg)
        else:
            print("[classic] actuator publisher not wired: override "
                  "publish_actuators() for your Gazebo transport", file=sys.stderr)

    def on_imu(msg) -> None:
        angular = getattr(msg, "angular_velocity", None)
        linear = getattr(msg, "linear_acceleration", None)
        if angular is None or linear is None:
            return
        gyro = axis_remap((angular.x, angular.y, angular.z), order, signs)
        accel = axis_remap((linear.x, linear.y, linear.z), order, signs)

        # seq MUST advance on every frame. HomeFly drops seq <= last_seq, so a
        # constant seq would discard every IMU frame after the first. This used
        # to stay at 1 whenever Gazebo supplied header.stamp (i.e. always in a
        # real Gazebo run), silently breaking the whole SITL lockstep.
        state["seq"] += 1
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None) if header is not None else None
        if stamp is not None and (getattr(stamp, "sec", 0) or getattr(stamp, "nsec", 0)):
            timestamp_us = int(stamp.sec) * 1_000_000 + int(stamp.nsec) // 1000
        else:
            timestamp_us = state["seq"] * args.period_us

        frame = make_imu(state["seq"], timestamp_us, gyro, accel)
        sock.sendto(frame, target)
        state["frames"] += 1

    if node is not None:
        ok = node.subscribe(imu_msg, args.imu_topic, on_imu)
        if not ok:
            raise RuntimeError(f"cannot subscribe to {args.imu_topic}")
    else:
        print("[classic] pygazebo subscription wiring depends on your Gazebo "
              "version; see docs/gazebo.md", file=sys.stderr)

    print(f"[{flavor}] bridge ready: {args.imu_topic} -> {target}, "
          f"motors <- :{args.motor_port} -> {args.motor_topic}")
    # --timeout is an IDLE timeout (no progress), not a total run deadline:
    # a long lockstep run can legitimately take longer than the default 5 s.
    last_progress = time.monotonic()

    while args.steps == 0 or state["motors"] < args.steps:
        if args.timeout and time.monotonic() - last_progress > args.timeout:
            print(f"[{flavor}] no motor frame for {args.timeout}s", file=sys.stderr)
            break
        motors = wait_motor(sock, 0.02)
        if motors is not None:
            drain_rx(sock)
            publish_actuators(motors)
            state["motors"] += 1
            last_progress = time.monotonic()

    sock.close()
    print(f"[{flavor}] done: imu_frames={state['frames']} "
          f"motor_frames={state['motors']}")
    return 0


# --------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="HomeFly <-> Gazebo SITL bridge (pure stdlib for mock)")
    parser.add_argument("--backend", choices=("mock", "gz", "classic"),
                        default="mock")
    parser.add_argument("--host", default="127.0.0.1",
                        help="host running HomeFly")
    parser.add_argument("--imu-port", type=int, default=14550,
                        help="HomeFly UDP port receiving IMU frames")
    parser.add_argument("--motor-port", type=int, default=14551,
                        help="bridge UDP port receiving motor frames")
    parser.add_argument("--steps", type=int, default=2000,
                        help="IMU/motor frames to exchange (0 = unlimited for gz)")
    parser.add_argument("--period-us", type=int, default=1000,
                        help="Gazebo sim time step; must match control_period_us")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="seconds without a motor frame before giving up")
    parser.add_argument("--initial-roll", type=float, default=0.20,
                        help="mock initial roll in rad")
    parser.add_argument("--initial-pitch", type=float, default=-0.12,
                        help="mock initial pitch in rad")
    parser.add_argument("--gyro-bias", type=float, default=0.0,
                        help="mock gyro bias in rad/s")
    parser.add_argument("--imu-topic", default="/world/quad/model/homefly/link/body/sensor/imu/imu",
                        help="Gazebo Garden IMU topic")
    parser.add_argument("--motor-topic", default="/actuators",
                        help="Gazebo Actuators topic (gz.msgs.Actuators)")
    parser.add_argument("--max-rot-velocity", type=float, default=1000.0,
                        help="maps normalized [0,1] thrust to rad/s Actuators.velocity")
    parser.add_argument("--axis-map", default="0,1,2",
                        help="gyro/accel axis permutation, e.g. 0,2,1")
    parser.add_argument("--axis-sign", default="1,1,1",
                        help="per-axis sign, e.g. 1,-1,1 for FLU->FRD")
    parser.add_argument("--self-test", action="store_true",
                        help="verify the wire protocol without networking")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return self_test()
    try:
        if args.backend == "mock":
            return run_mock(args)
        return run_gz(args, args.backend)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"gazebo_bridge: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
