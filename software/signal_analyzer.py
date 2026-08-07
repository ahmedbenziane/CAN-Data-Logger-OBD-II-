#!/usr/bin/env python3
"""
tools/signal_analyzer.py — CAN Capture Analyzer
=================================================
Reads the CSV produced by Node B passive sniffer.
Produces per-ID statistics: frame count, period, jitter,
byte range/entropy, rolling-counter detection, multi-byte candidates.

Usage
-----
  python tools/signal_analyzer.py capture.csv
  python tools/signal_analyzer.py capture.csv --id 0x7E8
  python tools/signal_analyzer.py capture.csv --id 0x100 --plot
  python tools/signal_analyzer.py capture.csv --export signals.json
  python tools/signal_analyzer.py capture.csv --window 30
  python tools/signal_analyzer.py capture.csv --no-color

CSV format
----------
  timestamp_us,id_hex,dlc,b0,b1,b2,b3,b4,b5,b6,b7

Lines starting with '#' are comments (skipped).
ESP-IDF log lines  "I (NNN) TAG: ..."  are stripped automatically.
PlatformIO monitor header lines are discarded automatically.
"""

import argparse
import json
import math
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ── Color support ────────────────────────────────────────────────────────────
# Disabled on Windows cmd.exe (no VIRTUAL_TERMINAL_PROCESSING by default)
# Always disabled when stdout is redirected to a file.
# Can be forced off with --no-color.

def _detect_color() -> bool:
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    # Windows cmd.exe does not support ANSI unless explicitly enabled
    if sys.platform == "win32":
        # Windows Terminal / VS Code terminal set TERM or WT_SESSION
        if os.environ.get("WT_SESSION") or os.environ.get("TERM"):
            return True
        # Try enabling VT processing (Windows 10+)
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            # ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
            handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
            mode   = ctypes.c_ulong()
            if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
                if kernel32.SetConsoleMode(handle, mode.value | 0x0004):
                    return True
        except Exception:
            pass
        return False
    return True

_COLOR = False  # set in main() after arg parsing

def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _COLOR else text

def BOLD(t):   return _c("1",  t)
def CYAN(t):   return _c("96", t)
def GREEN(t):  return _c("92", t)
def YELLOW(t): return _c("93", t)
def RED(t):    return _c("91", t)
def DIM(t):    return _c("2",  t)


# ── Data model ───────────────────────────────────────────────────────────────
class FrameRecord:
    __slots__ = ("ts_us", "dlc", "data")
    def __init__(self, ts_us: int, dlc: int, data: List[int]):
        self.ts_us = ts_us
        self.dlc   = dlc
        self.data  = data  # always length 8


# ── Regex patterns ───────────────────────────────────────────────────────────
# Matches ESP-IDF log prefix:  "I (11823) CSV: "  or  "(11763) CSV: "
_CSV_PREFIX_RE = re.compile(r"^[IWED]?\s*\(\d+\)\s+\w+:\s+")
# Matches any remaining ESP-IDF log line after prefix strip attempt
_ESP_LOG_RE    = re.compile(r"^[IWED] \(")
# PlatformIO monitor header lines
_PIO_SKIP_RE   = re.compile(
    r"^(Warning!|Please build|See https|---)")


# ── CSV loader ───────────────────────────────────────────────────────────────
def load_csv(path: str,
             filter_id: Optional[int] = None,
             window_s:  Optional[float] = None) -> Dict[int, List[FrameRecord]]:
    records:  Dict[int, List[FrameRecord]] = defaultdict(list)
    t0:       Optional[int] = None
    parsed    = 0
    stripped  = 0
    skipped   = 0

    with open(path, newline="", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line:
                continue

            # Discard PlatformIO monitor header
            if _PIO_SKIP_RE.match(line):
                continue

            # Discard comment lines and CSV header row
            if line.startswith("#") or line.startswith("timestamp"):
                continue

            # Strip ESP_LOGI prefix if still present
            m = _CSV_PREFIX_RE.match(line)
            if m:
                line = line[m.end():]
                stripped += 1

            # Discard remaining ESP log lines (SNIFFER/STATS tags)
            if _ESP_LOG_RE.match(line):
                continue

            parts = line.split(",")
            if len(parts) < 4:
                skipped += 1
                continue

            try:
                ts_us  = int(parts[0])
                can_id = int(parts[1], 16)
                dlc    = int(parts[2])
                data   = [int(x, 16) for x in parts[3:11]]
                while len(data) < 8:
                    data.append(0)
            except (ValueError, IndexError):
                skipped += 1
                continue

            # Time window
            if t0 is None:
                t0 = ts_us
            if window_s is not None and (ts_us - t0) > window_s * 1_000_000:
                break

            # ID filter
            if filter_id is not None and can_id != filter_id:
                continue

            records[can_id].append(FrameRecord(ts_us, dlc, data))
            parsed += 1

    # Build status line
    status = f"Parsed {parsed} frames, {len(records)} unique IDs"
    if stripped:
        status += f", {stripped} ESP_LOGI prefixes stripped"
        status += " (firmware fix: use printf not ESP_LOGI for CSV lines)"
    if skipped:
        status += f", {skipped} unparseable lines skipped"
    print(DIM(f"  {status}"))
    return dict(records)


# ── Statistics helpers ────────────────────────────────────────────────────────
def mean_std(values: List[float]) -> Tuple[float, float]:
    if not values:
        return 0.0, 0.0
    n  = len(values)
    mu = sum(values) / n
    var = sum((x - mu) ** 2 for x in values) / n if n > 1 else 0.0
    return mu, math.sqrt(var)


def shannon_entropy(col: List[int]) -> float:
    if not col:
        return 0.0
    counts: Dict[int, int] = defaultdict(int)
    for v in col:
        counts[v] += 1
    n = len(col)
    h = 0.0
    for c in counts.values():
        p  = c / n
        h -= p * math.log2(p)
    return h


def detect_rolling(col: List[int]) -> Optional[int]:
    """Return +1/-1 if column is a consistent rolling counter, else None."""
    if len(col) < 8:
        return None
    deltas = [(col[i] - col[i-1]) & 0xFF for i in range(1, len(col))]
    for raw, label in [(1, +1), (255, -1)]:
        if sum(1 for d in deltas if d == raw) / len(deltas) >= 0.90:
            return label
    return None


def detect_multibyte(frames: List[FrameRecord]) -> List[Tuple[int, int, str]]:
    if len(frames) < 4:
        return []
    results = []
    for hi in range(7):
        lo = hi + 1
        combined = [(frames[i].data[hi] << 8) | frames[i].data[lo]
                    for i in range(len(frames))]
        rng = max(combined) - min(combined)
        if rng > 512:
            results.append((hi, lo,
                f"16-bit candidate  range {min(combined):#06x}–{max(combined):#06x}"
                f"  ({rng} counts)"))
    return results


def entropy_bar(h: float) -> str:
    filled = min(8, round(h))
    return f"[{'█' * filled}{'░' * (8 - filled)}]"


# ── Per-ID report ─────────────────────────────────────────────────────────────
def report_id(can_id: int, frames: List[FrameRecord]) -> dict:
    n = len(frames)
    periods = [(frames[i].ts_us - frames[i-1].ts_us) / 1000.0
               for i in range(1, n)]
    p_mean, p_std = mean_std(periods)
    p_min  = min(periods) if periods else 0.0
    p_max  = max(periods) if periods else 0.0
    dur_s  = (frames[-1].ts_us - frames[0].ts_us) / 1_000_000 if n > 1 else 0.0

    print()
    print(BOLD(CYAN(f"ID 0x{can_id:03X}")) +
          f"  |  {n} frames"
          f"  |  period={p_mean:.1f}ms +/-{p_std:.2f}ms"
          f"  (min={p_min:.1f}ms  max={p_max:.1f}ms)"
          f"  |  {dur_s:.1f}s captured")

    dlc = frames[0].dlc
    byte_info = []

    for b in range(dlc):
        col      = [f.data[b] for f in frames]
        lo, hi   = min(col), max(col)
        entropy  = shannon_entropy(col)
        unique_n = len(set(col))
        rolling  = detect_rolling(col)

        annotations = []
        if lo == hi:
            annotations.append(GREEN("constant"))
        elif rolling == +1:
            annotations.append(YELLOW("rolling counter +1/frame"))
        elif rolling == -1:
            annotations.append(YELLOW("rolling counter -1/frame"))
        elif lo == 0x00 and hi == 0xFF:
            annotations.append("full range  <- high byte of signal?")
        elif entropy > 6.0:
            annotations.append("wide range  <- likely signal data")

        note = "  <- " + ", ".join(annotations) if annotations else ""
        bar  = entropy_bar(entropy)

        print(f"  Byte {b}: range [{lo:#04x}-{hi:#04x}]"
              f"  unique={unique_n:3d}"
              f"  entropy={entropy:4.1f}b  {bar}{note}")

        byte_info.append({
            "byte": b, "min": lo, "max": hi,
            "unique": unique_n, "entropy": round(entropy, 2),
            "rolling": rolling,
        })

    for hi_b, lo_b, note in detect_multibyte(frames):
        print(f"  {YELLOW('-> Multi-byte:')} bytes [{hi_b},{lo_b}]  {note}")

    return {
        "id": f"0x{can_id:03X}",
        "frame_count": n,
        "period_ms": round(p_mean, 2),
        "period_std_ms": round(p_std, 2),
        "duration_s": round(dur_s, 2),
        "dlc": dlc,
        "bytes": byte_info,
    }


# ── Plot ──────────────────────────────────────────────────────────────────────
def plot_id(can_id: int, frames: List[FrameRecord]) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError:
        print(RED("matplotlib not installed  ->  pip install matplotlib"))
        return

    dlc = frames[0].dlc
    ts  = [(f.ts_us - frames[0].ts_us) / 1_000_000 for f in frames]

    fig, axes = plt.subplots(dlc, 1, figsize=(14, 2 * dlc), sharex=True)
    if dlc == 1:
        axes = [axes]

    fig.suptitle(f"CAN ID 0x{can_id:03X}  -  {len(frames)} frames", fontsize=13)
    for b, ax in enumerate(axes):
        col = [f.data[b] for f in frames]
        ax.plot(ts, col, linewidth=0.7, color=f"C{b}")
        ax.set_ylabel(f"Byte {b}", fontsize=8)
        ax.yaxis.set_major_formatter(
            ticker.FuncFormatter(lambda x, _: f"0x{int(x):02X}"))
        ax.set_ylim(-5, 265)
        ax.grid(True, linewidth=0.3)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    global _COLOR

    ap = argparse.ArgumentParser(
        description="CAN capture analyzer — Renault Clio IV / ESP32 sniffer")
    ap.add_argument("csv",          help="Input capture.csv file")
    ap.add_argument("--id",         help="Focus on one CAN ID, e.g. 0x7E8")
    ap.add_argument("--window",     type=float,
                                    help="Analyse only first N seconds of capture")
    ap.add_argument("--plot",       action="store_true",
                                    help="Plot byte values over time (needs --id + matplotlib)")
    ap.add_argument("--export",     metavar="FILE",
                                    help="Export analysis to JSON file")
    ap.add_argument("--min-frames", type=int, default=2,
                                    help="Skip IDs with fewer than N frames (default 2)")
    ap.add_argument("--no-color",   action="store_true",
                                    help="Disable ANSI colors (auto-detected on Windows cmd)")
    args = ap.parse_args()

    # Set color: respect --no-color, otherwise auto-detect terminal support
    _COLOR = False if args.no_color else _detect_color()

    if not Path(args.csv).exists():
        print(f"ERROR: File not found: {args.csv}")
        sys.exit(1)

    filter_id: Optional[int] = None
    if args.id:
        try:
            filter_id = int(args.id, 16)
        except ValueError:
            print(f"ERROR: Invalid CAN ID '{args.id}' — use hex e.g. 0x7E8")
            sys.exit(1)

    print(BOLD("\n=== CAN Signal Analyzer ==="))
    print(DIM(f"  Input : {args.csv}"))
    if filter_id is not None:
        print(DIM(f"  Filter: 0x{filter_id:03X}"))
    if args.window:
        print(DIM(f"  Window: {args.window}s"))

    records = load_csv(args.csv, filter_id=filter_id, window_s=args.window)

    if not records:
        print("No matching frames found.")
        print()
        print("Checklist:")
        print("  1. Is Node A transmitting?  pio device monitor --environment node_a")
        print("  2. Are CANH / CANL / GND wired between both nodes?")
        print("  3. Is there a 120 ohm terminator at each end?")
        print("  4. Did you wait >5 seconds before Ctrl+C?")
        sys.exit(0)

    export_data = []
    for can_id in sorted(records.keys()):
        frames = records[can_id]
        if len(frames) < args.min_frames:
            continue
        export_data.append(report_id(can_id, frames))

    total = sum(len(v) for v in records.values())
    print()
    print(BOLD(f"--- Summary: {len(export_data)} IDs / {total} total frames ---"))

    # Jitter verdict
    print()
    for entry in export_data:
        jitter = entry["period_std_ms"]
        period = entry["period_ms"]
        if period > 0:
            pct = (jitter / period) * 100
            if pct < 5:
                verdict = GREEN("GOOD")
            elif pct < 20:
                verdict = YELLOW("ACCEPTABLE")
            else:
                verdict = RED("HIGH - firmware printf not flushing fast enough")
            print(f"  ID 0x{int(entry['id'], 16):03X}  jitter={jitter:.2f}ms"
                  f"  ({pct:.1f}% of period)  {verdict}")

    if args.plot:
        if filter_id is None:
            print(YELLOW("--plot requires --id to select a single CAN ID"))
        elif filter_id in records:
            plot_id(filter_id, records[filter_id])
        else:
            print(f"ID 0x{filter_id:03X} not found in capture")

    if args.export:
        out = {
            "source": args.csv,
            "filter_id": f"0x{filter_id:03X}" if filter_id else "all",
            "total_frames": total,
            "ids": export_data,
        }
        with open(args.export, "w") as f:
            json.dump(out, f, indent=2)
        print(GREEN(f"  Exported -> {args.export}"))


if __name__ == "__main__":
    main()