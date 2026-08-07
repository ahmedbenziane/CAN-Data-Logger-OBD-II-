#!/usr/bin/env python3
"""
tools/decode_log.py — CAN candump-log decoder
=============================================
Reads a candump-format log (the format produced by check_log.py):

    (3.127528) vcan0 12E#C67FFE7FF0FFFF00

Produces a sorted table of every CAN ID with frame counts and an example
payload. If a DBC file is supplied (--dbc), each ID's example frame is
decoded into named signals with physical values. Everything can be exported
to CSV (--csv) for downstream feature extraction.

Usage
-----
  python decode_log.py pure_can.txt
  python decode_log.py pure_can.txt --dbc renault_clio4.dbc
  python decode_log.py pure_can.txt --dbc renault_clio4.dbc --csv signals.csv
  python decode_log.py pure_can.txt --probe 0x181            # raw 16-bit probe

The DBC features require `cantools` (pip install cantools). The summary
table and CSV export work without it.
"""

import argparse
import csv
import sys
from collections import defaultdict

# Windows consoles often default to cp1252, which can't encode the status
# emoji below. Switch stdout/stderr to UTF-8 where the runtime supports it.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

# candump line:  (3.127528) vcan0 12E#C67FFE7FF0FFFF00
import re

LINE_RE = re.compile(r"\(\s*(\d+\.\d+)\s*\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")


def parse_log(path):
    """Yield (timestamp_float, can_id_int, data_bytes) for each frame."""
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            ts, id_hex, data_hex = m.groups()
            # data_hex may be odd length on a corrupt frame; trim to even
            if len(data_hex) % 2:
                data_hex = data_hex[:-1]
            yield float(ts), int(id_hex, 16), bytes.fromhex(data_hex)


def load_dbc(path):
    try:
        import cantools
    except ImportError:
        print("⚠  cantools not installed — DBC decoding disabled. "
              "Run: pip install cantools", file=sys.stderr)
        return None
    try:
        return cantools.database.load_file(path)
    except Exception as e:  # noqa: BLE001 - surface any DBC load error to user
        print(f"⚠  Could not load DBC '{path}': {e}", file=sys.stderr)
        return None


def decode_frame(db, can_id, data):
    """Return a short 'Sig=val, ...' string, or '' if undecodable."""
    if db is None:
        return ""
    try:
        decoded = db.decode_message(can_id, data)
    except Exception:  # noqa: BLE001 - unknown id / bad length / scaling
        return ""
    parts = []
    for name, val in decoded.items():
        if isinstance(val, float):
            parts.append(f"{name}={val:g}")
        else:
            parts.append(f"{name}={val}")
    return ", ".join(parts)


def classify_byte(values):
    """Heuristic label for one byte position given all its observed values.

    Returns (label, distinct_count, min, max).
    """
    distinct = len(set(values))
    lo, hi = min(values), max(values)
    if distinct == 1:
        return "const", distinct, lo, hi
    # Rolling counter: consecutive deltas are almost always +1 within a fixed
    # modulus (commonly 0..15 in the low nibble, or full 0..255).
    deltas = [(b - a) for a, b in zip(values, values[1:])]
    inc = sum(1 for d in deltas if d in (1, -15, -255, -(hi - lo)))
    if deltas and inc / len(deltas) > 0.8:
        return "counter", distinct, lo, hi
    if distinct <= 4:
        return "enum/flags", distinct, lo, hi
    return "LIVE", distinct, lo, hi


def analyze(per_id_bytes, only_id=None):
    """Print per-byte variability so live signal bytes stand out."""
    print("\nByte-level variability  (LIVE = likely a real signal)\n")
    for can_id in sorted(per_id_bytes):
        if only_id is not None and can_id != only_id:
            continue
        frames = per_id_bytes[can_id]
        if not frames:
            continue
        width = max(len(f) for f in frames)
        print(f"0x{can_id:03X}  ({len(frames)} frames)")
        for pos in range(width):
            col = [f[pos] for f in frames if len(f) > pos]
            if not col:
                continue
            label, distinct, lo, hi = classify_byte(col)
            bar = "  <<<" if label == "LIVE" else ""
            print(f"   b{pos}: {label:<10} distinct={distinct:<4} "
                  f"range=0x{lo:02X}..0x{hi:02X}{bar}")
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Decode a candump CAN log.")
    ap.add_argument("logfile", help="candump log (e.g. pure_can.txt)")
    ap.add_argument("--dbc", help="optional DBC file for signal decoding")
    ap.add_argument("--csv", help="export every frame to this CSV file")
    ap.add_argument("--probe", help="hex CAN ID to print raw 16-bit b0:b1 value, "
                                    "e.g. 0x181")
    ap.add_argument("--analyze", nargs="?", const="ALL", metavar="ID",
                    help="per-byte variability for all IDs, or one ID "
                         "(e.g. --analyze 0x18A). Finds live signal bytes "
                         "without a DBC.")
    ap.add_argument("--track", metavar="ID:BYTE",
                    help="dump a time series 'timestamp value' for one byte, "
                         "e.g. --track 0x18A:0 . Pipe to a file and plot it.")
    args = ap.parse_args(argv)

    db = load_dbc(args.dbc) if args.dbc else None
    probe_id = int(args.probe, 16) if args.probe else None

    analyze_id = None
    if args.analyze and args.analyze != "ALL":
        analyze_id = int(args.analyze, 16)

    track_id = track_byte = None
    if args.track:
        id_part, byte_part = args.track.split(":")
        track_id, track_byte = int(id_part, 16), int(byte_part)

    counts = defaultdict(int)
    example = {}            # can_id -> example data bytes
    per_id_bytes = defaultdict(list)   # can_id -> [bytes, ...] for analysis
    total = 0

    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "can_id", "data_hex", "decoded_signals"])

    for ts, can_id, data in parse_log(args.logfile):
        total += 1
        counts[can_id] += 1
        if can_id not in example:
            example[can_id] = data

        if args.analyze:
            if analyze_id is None or can_id == analyze_id:
                per_id_bytes[can_id].append(data)

        if track_id is not None and can_id == track_id and len(data) > track_byte:
            print(f"{ts:.6f} {data[track_byte]}")

        if probe_id is not None and can_id == probe_id and len(data) >= 2:
            raw = int.from_bytes(data[0:2], "big")
            print(f"probe 0x{can_id:03X}: b0:b1=0x{raw:04X} ({raw}) "
                  f"*0.25={raw * 0.25:g}")

        if csv_writer is not None:
            decoded = decode_frame(db, can_id, data)
            csv_writer.writerow([f"{ts:.6f}", f"0x{can_id:03X}",
                                 data.hex().upper(), decoded])

    if csv_file is not None:
        csv_file.close()

    # --track only emits the time series (already printed above) for piping.
    if track_id is not None:
        return

    if args.analyze:
        analyze(per_id_bytes, analyze_id)
        return

    print(f"\n📊 Loaded {total} frames across {len(counts)} unique IDs.\n")
    print(f"{'ID':<8}{'Count':>8}  {'Example Data':<26}{'Decoded (if DBC)'}")
    print("-" * 90)
    for can_id in sorted(counts):
        data = example[can_id]
        ex = " ".join(f"{b:02X}" for b in data)
        decoded = decode_frame(db, can_id, data)
        print(f"0x{can_id:03X}  {counts[can_id]:>8}  {ex:<26}{decoded}")

    if args.csv:
        print(f"\n💾 Wrote per-frame CSV → {args.csv}")


if __name__ == "__main__":
    main()
