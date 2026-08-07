#!/usr/bin/env python3
"""
can_sniffer.py  —  Real-time CAN bus differential viewer for ESP32 + MCP2515

Reads candump-format lines from an ESP32 serial port (firmware built with
-DCANDUMP_OUTPUT) and displays a live terminal table.  Changed data bytes
are highlighted yellow so signal boundaries jump out during reverse
engineering of proprietary Renault HS-CAN frames.

Usage:
    python can_sniffer.py --port COM12 --baud 115200
    python can_sniffer.py --port /dev/ttyUSB0
    python can_sniffer.py --file  capture.log          # replay saved log
    cat capture.log | python can_sniffer.py --stdin    # pipe

Firmware candump output format (one frame per line):
    (1234567.890123) vcan0 0A6#0102030405060708

Also accepts the firmware JSON OBD dashboard lines (shows 0x7E8 as OBD).

Dependencies:
    pip install pyserial          # for --port
    pip install windows-curses    # Windows only (curses not built-in)
"""

import argparse
import sys
import time
import threading
import json
from collections import deque

try:
    import curses
except ImportError:
    sys.exit(
        "curses not available.\n"
        "On Windows install it with:  pip install windows-curses\n"
        "Then re-run the script."
    )

try:
    import serial
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False


# ── Per-ID state ───────────────────────────────────────────────────────────

class FrameEntry:
    __slots__ = ('id', 'dlc', 'data', 'changed_mask',
                 'count', 'ts_history', 'last_ts', 'first_ts')

    def __init__(self, can_id: int, dlc: int, data: list, ts: float):
        self.id           = can_id
        self.dlc          = dlc
        self.data         = data[:]
        self.changed_mask = 0xFF        # all bytes "new" on first appearance
        self.count        = 1
        self.ts_history   = deque(maxlen=40)
        self.ts_history.append(ts)
        self.last_ts      = ts
        self.first_ts     = ts

    def update(self, dlc: int, data: list, ts: float) -> None:
        self.changed_mask = 0
        for i in range(min(dlc, self.dlc, 8)):
            if data[i] != self.data[i]:
                self.changed_mask |= (1 << i)
        self.dlc   = dlc
        self.data  = data[:]
        self.count += 1
        self.ts_history.append(ts)
        self.last_ts = ts

    def rate_hz(self, now: float) -> float:
        """Average frames/s over the last 2 seconds."""
        cutoff = now - 2.0
        recent = sum(1 for t in self.ts_history if t >= cutoff)
        return recent / 2.0


# ── Frame parsers ──────────────────────────────────────────────────────────

def parse_candump(line: str):
    """
    Parse one candump line:
        (1234567.890123) vcan0 0A6#0102030405060708
    Returns (timestamp_s, can_id, dlc, data_list) or None.
    """
    line = line.strip()
    if not line or '#' not in line:
        return None
    try:
        parts = line.split()
        if len(parts) < 3:
            return None
        ts    = float(parts[0].strip('()'))
        frame = parts[2]
        id_str, data_str = frame.split('#', 1)
        can_id = int(id_str, 16)
        raw    = bytes.fromhex(data_str) if data_str else b''
        dlc    = len(raw)
        data   = list(raw) + [0] * (8 - dlc)
        return ts, can_id, dlc, data
    except (ValueError, IndexError):
        return None


def parse_json_obd(line: str):
    """Parse JSON dashboard line — synthesise a 0x7E8 entry for OBD stats."""
    line = line.strip()
    if not line.startswith('{'):
        return None
    try:
        obj = json.loads(line)
        if obj.get('type') != 'obd':
            return None
        ts = obj.get('t', 0) / 1000.0
        # Pack RPM and speed into a pseudo-frame so they show up in the table
        rpm   = int(obj.get('rpm', 0))
        spd   = int(obj.get('spd', 0))
        data  = [rpm >> 8, rpm & 0xFF, spd, 0, 0, 0, 0, 0]
        return ts, 0x7E8, 3, data
    except (json.JSONDecodeError, KeyError):
        return None


# ── Shared state ───────────────────────────────────────────────────────────

_lock        = threading.Lock()
_frames: dict[int, FrameEntry] = {}
_total_count = 0
_running     = True


def _ingest(line: str) -> None:
    global _total_count
    result = parse_candump(line) or parse_json_obd(line)
    if result is None:
        return
    ts, can_id, dlc, data = result
    if ts == 0.0:
        ts = time.monotonic()
    with _lock:
        if can_id in _frames:
            _frames[can_id].update(dlc, data, ts)
        else:
            _frames[can_id] = FrameEntry(can_id, dlc, data, ts)
        _total_count += 1


# ── Reader threads ─────────────────────────────────────────────────────────

def _serial_reader(port: str, baud: int) -> None:
    global _running
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        _running = False
        print(f"Serial error: {exc}", file=sys.stderr)
        return
    while _running:
        try:
            raw = ser.readline()
            if raw:
                _ingest(raw.decode('ascii', errors='replace'))
        except serial.SerialException:
            break
    ser.close()


def _file_reader(path: str) -> None:
    global _running
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            for line in fh:
                if not _running:
                    break
                _ingest(line)
                time.sleep(0.0001)   # pace replay to ~10k lines/s
    except OSError as exc:
        print(f"File error: {exc}", file=sys.stderr)
    _running = False


def _stdin_reader() -> None:
    global _running
    for line in sys.stdin:
        if not _running:
            break
        _ingest(line)
    _running = False


# ── Curses UI ──────────────────────────────────────────────────────────────

_C_NORMAL  = 1   # white
_C_CHANGED = 2   # yellow — byte changed since last frame
_C_NEW     = 3   # cyan   — ID appeared for the first time
_C_STALE   = 4   # dim    — no update in >3 s
_C_HEADER  = 5   # green  — column header / footer bar

_STALE_SEC = 3.0


def _ui(stdscr) -> None:
    global _running

    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(_C_NORMAL,  curses.COLOR_WHITE,  -1)
    curses.init_pair(_C_CHANGED, curses.COLOR_YELLOW, -1)
    curses.init_pair(_C_NEW,     curses.COLOR_CYAN,   -1)
    try:
        curses.init_pair(_C_STALE, 8, -1)   # colour 8 = dark/bright-black
    except Exception:
        curses.init_pair(_C_STALE, curses.COLOR_BLACK, -1)
    curses.init_pair(_C_HEADER,  curses.COLOR_GREEN,  -1)

    while _running:
        ch = stdscr.getch()
        if ch in (ord('q'), ord('Q'), 27):
            _running = False
            break

        h, w = stdscr.getmaxyx()
        stdscr.erase()
        now = time.monotonic()

        # ── title bar ──────────────────────────────────────────────────
        title = (f"  CAN Sniffer  |  frames={_total_count}  "
                 f"ids={len(_frames)}  |  q=quit")
        stdscr.addstr(0, 0, title[:w - 1],
                      curses.color_pair(_C_HEADER) | curses.A_BOLD)

        # ── column header ──────────────────────────────────────────────
        col_hdr = (f"  {'ID':>5}  {'Hz':>5}  {'count':>7}  "
                   f"B0  B1  B2  B3  B4  B5  B6  B7  DLC")
        stdscr.addstr(1, 0, col_hdr[:w - 1], curses.A_UNDERLINE)

        # ── rows ───────────────────────────────────────────────────────
        with _lock:
            rows = sorted(_frames.values(), key=lambda e: e.id)

        r = 2
        for e in rows:
            if r >= h - 1:
                break
            age  = now - e.last_ts
            rate = e.rate_hz(now)

            row_attr = (curses.color_pair(_C_STALE)
                        if age > _STALE_SEC
                        else curses.color_pair(_C_NORMAL))

            prefix = f"  {e.id:05X}  {rate:5.1f}  {e.count:7d}  "
            try:
                stdscr.addstr(r, 0, prefix[:w - 1], row_attr)
            except curses.error:
                pass

            col = len(prefix)
            for i in range(8):
                if col + 3 >= w:
                    break
                if i < e.dlc:
                    bstr = f"{e.data[i]:02X} "
                    if age > _STALE_SEC:
                        battr = curses.color_pair(_C_STALE)
                    elif e.changed_mask & (1 << i):
                        battr = curses.color_pair(_C_CHANGED) | curses.A_BOLD
                    else:
                        battr = curses.color_pair(_C_NORMAL)
                else:
                    bstr  = ".. "
                    battr = curses.color_pair(_C_STALE)
                try:
                    stdscr.addstr(r, col, bstr, battr)
                except curses.error:
                    pass
                col += 3

            dlc_str = f" {e.dlc}"
            try:
                stdscr.addstr(r, col, dlc_str[:w - col - 1], row_attr)
            except curses.error:
                pass

            r += 1

        # ── footer ─────────────────────────────────────────────────────
        footer = " " + "─" * max(0, w - 2)
        try:
            stdscr.addstr(h - 1, 0, footer[:w - 1],
                          curses.color_pair(_C_HEADER))
        except curses.error:
            pass

        stdscr.refresh()
        time.sleep(0.05)    # 20 fps


# ── Entry point ────────────────────────────────────────────────────────────

def main() -> None:
    global _running

    ap = argparse.ArgumentParser(
        description='Real-time CAN differential viewer for ESP32 serial output')
    src = ap.add_mutually_exclusive_group()
    src.add_argument('--port',  metavar='PORT',
                     help='Serial port  (e.g. COM3, /dev/ttyUSB0)')
    src.add_argument('--file',  metavar='FILE',
                     help='Replay a saved candump log')
    src.add_argument('--stdin', action='store_true',
                     help='Read from stdin (pipe mode)')
    ap.add_argument('--baud', type=int, default=115200,
                    help='Serial baud rate (default: 115200)')
    args = ap.parse_args()

    if args.port and not HAVE_SERIAL:
        sys.exit('pyserial not installed — run: pip install pyserial')

    if args.port:
        t = threading.Thread(target=_serial_reader,
                             args=(args.port, args.baud), daemon=True)
    elif args.file:
        t = threading.Thread(target=_file_reader,
                             args=(args.file,), daemon=True)
    else:
        t = threading.Thread(target=_stdin_reader, daemon=True)

    t.start()

    try:
        curses.wrapper(_ui)
    finally:
        _running = False


if __name__ == '__main__':
    main()
