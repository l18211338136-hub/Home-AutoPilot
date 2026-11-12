#!/usr/bin/env python3
"""HomeFly Flight Log (.hfl) offline reader / verifier.

Pure standard library. Usage:

    python tools/hfl_dump.py homefly_flight.hfl
    python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv
    python tools/hfl_dump.py homefly_flight.hfl --tail 10

Format v1 (all little endian, see docs/flight_log.md):
    [ header 64 B ] [ N * record 72 B ] [ footer 32 B ]

The footer is optional in the sense that a crash / power loss leaves it
missing; in that case the tool still parses all complete records and reports
the session as truncated.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from pathlib import Path

HEADER = struct.Struct("<IHHQQIII16s3I")  # 64 B
FOOTER = struct.Struct("<IIQIIII")  # 32 B
RECORD = struct.Struct("<IBBHQ13f")  # 68 B meaningful bytes (+ padding)

HEADER_MAGIC = 0x48464C47  # "HFLG"
FOOTER_MAGIC = 0x48464C45  # "HFLE"
RECORD_MAGIC = 0x48464644  # "HFFD"
VERSION = 1
FNV_OFFSET = 2166136261
FNV_PRIME = 16777619
RAD_TO_DEG = 180.0 / math.pi
MASK32 = 0xFFFFFFFF

assert HEADER.size == 64
assert FOOTER.size == 32
assert RECORD.size == 68


def fnv1a(data: bytes, seed: int) -> int:
    h = seed
    for byte in data:
        h = ((h ^ byte) * FNV_PRIME) & MASK32
    return h


def read_header(blob: bytes):
    if len(blob) < HEADER.size:
        raise ValueError("file is shorter than a flight log header")
    magic, version, header_size, session_id, start_us, record_size, period_us, \
        config_hash, platform, *_reserved = HEADER.unpack_from(blob, 0)
    if magic != HEADER_MAGIC:
        raise ValueError(f"bad header magic 0x{magic:08X} (not a HomeFly flight log)")
    if version != VERSION:
        raise ValueError(f"unsupported flight log version {version}")
    if header_size != HEADER.size:
        raise ValueError(f"bad header size {header_size}, expected {HEADER.size}")
    if record_size < RECORD.size:
        raise ValueError(f"bad record size {record_size}, expected >= {RECORD.size}")
    return {
        "session_id": session_id,
        "start_time_us": start_us,
        "record_size": record_size,
        "control_period_us": period_us,
        "config_hash": config_hash,
        "platform": platform.split(b"\0", 1)[0].decode("ascii", "replace"),
    }


def parse(blob: bytes):
    header = read_header(blob)
    body = blob[HEADER.size:]
    footer = None

    if len(body) >= FOOTER.size:
        candidate = FOOTER.unpack(body[-FOOTER.size:])
        if candidate[0] == FOOTER_MAGIC:
            footer = {
                "record_count": candidate[1],
                "end_time_us": candidate[2],
                "dropped_records": candidate[3],
                "write_failures": candidate[4],
                "checksum": candidate[5],
            }
            body = body[:-FOOTER.size]

    step = header["record_size"]
    truncated = len(body) % step
    count = len(body) // step
    checksum = FNV_OFFSET
    records = []
    bad_records = 0

    for i in range(count):
        raw = body[i * step:(i + 1) * step]
        magic, version, mode, _reserved, timestamp_us, *values = RECORD.unpack_from(raw)
        if magic != RECORD_MAGIC or version != VERSION:
            bad_records += 1
            continue
        checksum = fnv1a(raw, checksum)
        records.append((timestamp_us, mode, values))

    return header, footer, records, checksum, truncated, bad_records


def write_csv(path: Path, records) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "timestamp_us", "t_s", "mode",
            "roll_deg", "pitch_deg", "yaw_deg",
            "gyro_x", "gyro_y", "gyro_z",
            "accel_x", "accel_y", "accel_z",
            "motor_1", "motor_2", "motor_3", "motor_4",
        ])
        for timestamp_us, mode, v in records:
            writer.writerow([
                timestamp_us, f"{timestamp_us * 1e-6:.6f}", mode,
                f"{v[0] * RAD_TO_DEG:.4f}", f"{v[1] * RAD_TO_DEG:.4f}",
                f"{v[2] * RAD_TO_DEG:.4f}",
                *[f"{x:.6f}" for x in v[3:6]],
                *[f"{x:.6f}" for x in v[6:9]],
                *[f"{x:.6f}" for x in v[9:13]],
            ])


def main() -> int:
    parser = argparse.ArgumentParser(description="Dump / verify a HomeFly .hfl flight log")
    parser.add_argument("file", type=Path, help="path to the .hfl file")
    parser.add_argument("--csv", type=Path, help="export records to CSV")
    parser.add_argument("--tail", type=int, default=0, metavar="N",
                        help="print the last N records")
    parser.add_argument("--strict", action="store_true",
                        help="also fail when frames were dropped (useful in CI)")
    args = parser.parse_args()

    if not args.file.is_file():
        print(f"error: {args.file} not found", file=sys.stderr)
        return 2

    try:
        header, footer, records, checksum, truncated, bad_records = parse(args.file.read_bytes())
    except (ValueError, struct.error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    ok = True
    print(f"file            : {args.file} ({args.file.stat().st_size} bytes)")
    print(f"platform        : {header['platform']}  session=0x{header['session_id']:016X}")
    print(f"control period  : {header['control_period_us']} us "
          f"({1_000_000 / header['control_period_us']:.0f} Hz)")
    print(f"config hash     : 0x{header['config_hash']:08X}")
    print(f"record size     : {header['record_size']} B   records={len(records)}")
    if footer:
        print(f"footer          : count={footer['record_count']} "
              f"dropped={footer['dropped_records']} "
              f"write_failures={footer['write_failures']}")
        print(f"checksum        : file=0x{footer['checksum']:08X} "
              f"computed=0x{checksum:08X}")
        if footer["checksum"] != checksum:
            print("  !! checksum mismatch - log is corrupt", file=sys.stderr)
            ok = False
        if footer["record_count"] != len(records):
            print("  !! record count mismatch", file=sys.stderr)
            ok = False
        if footer["write_failures"]:
            print("  !! storage reported write failures", file=sys.stderr)
            ok = False
        if footer["dropped_records"]:
            note = f"  !! {footer['dropped_records']} frames dropped by the ring buffer"
            if args.strict:
                print(note + " (--strict)", file=sys.stderr)
                ok = False
            else:
                print(note + " - log is readable but not gap free", file=sys.stderr)
    else:
        print("footer          : MISSING (crash / power loss / unfinished session)")
        ok = False

    if truncated:
        print(f"  !! {truncated} trailing bytes do not form a full record", file=sys.stderr)
        ok = False
    if bad_records:
        print(f"  !! {bad_records} records with bad magic/version", file=sys.stderr)
        ok = False

    if records:
        duration = (records[-1][0] - records[0][0]) * 1e-6
        print(f"duration        : {duration:.3f} s "
              f"({records[0][0]} .. {records[-1][0]} us)")
        print(f"first / last    : roll {records[0][2][0] * RAD_TO_DEG:7.3f} -> "
              f"{records[-1][2][0] * RAD_TO_DEG:7.3f} deg, "
              f"pitch {records[0][2][1] * RAD_TO_DEG:7.3f} -> "
              f"{records[-1][2][1] * RAD_TO_DEG:7.3f} deg")

    if args.tail > 0:
        print("\nlast records (t_ms mode roll pitch yaw deg):")
        for timestamp_us, mode, v in records[-args.tail:]:
            print(f"  [{timestamp_us // 1000:8d}] mode={mode} "
                  f"roll={v[0] * RAD_TO_DEG:8.3f} pitch={v[1] * RAD_TO_DEG:8.3f} "
                  f"yaw={v[2] * RAD_TO_DEG:8.3f}")

    if args.csv:
        write_csv(args.csv, records)
        print(f"\nCSV written     : {args.csv} ({len(records)} rows)")

    print(f"\nverification    : {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
