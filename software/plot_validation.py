#!/usr/bin/env python3
"""
plot_validation.py  —  Generate thesis validation figures from logged sessions.

Consumes the artefacts produced by can_logger.py:
    session_YYYYmmdd_HHMMSS/
        raw.jsonl        <- car-mode OBD lines  {"type":"obd", "rpm":..., "req":...}
        decoded.csv      <- bench decoded snapshots (rpm, steer, wheels, brake)
        uds.csv          <- UDS DID read results
        gps.csv          <- optional GPS fixes

Produces 300-DPI PNG figures ready to drop into the thesis:
    Plot 14.3  rpm_vs_time.png
    Plot 14.4  speed_vs_time.png
    Plot 14.5  coolant_warmup.png
    Plot 14.6  multi_signal.png
    Plot 14.7  steering_vs_time.png   (bench / sniff)
    Plot 14.8  wheel_speeds.png       (bench / sniff)
    Plot 14.9  brake_pressure.png     (bench / sniff)
    Plot 14.10 obd_success_rate.png
    Plot 14.1  stream_rate_bar.png    (bench, from decoded.csv arrival rate)

Usage:
    pip install matplotlib
    python plot_validation.py logs/session_20260614_101530
    python plot_validation.py logs/session_20260614_101530 --outdir figures
    python plot_validation.py --demo            # synthesise data, prove the plots

All plots degrade gracefully: if a signal is absent from the log, that figure
is skipped with a printed note rather than crashing.
"""

import argparse
import csv
import json
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")            # headless — no display needed
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib not installed — run: pip install matplotlib")


# ── Thesis-consistent figure style ──────────────────────────────────────────
plt.rcParams.update({
    "figure.figsize":  (8, 4.2),
    "figure.dpi":      120,
    "savefig.dpi":     300,
    "font.size":       11,
    "axes.grid":       True,
    "grid.alpha":      0.3,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "lines.linewidth": 1.4,
})


# ── Loaders ─────────────────────────────────────────────────────────────────

def load_obd(session):
    """Read car-mode OBD records from raw.jsonl. Returns list of dicts."""
    path = os.path.join(session, "raw.jsonl")
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                continue
            if m.get("type") == "obd":
                out.append(m)
    return out


def load_decoded(session):
    """Read bench decoded.csv. Returns list of dicts with numeric fields."""
    path = os.path.join(session, "decoded.csv")
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                out.append({
                    "t":     float(row["timestamp_ms"]),
                    "rpm":   float(row["rpm"]),
                    "steer": float(row["steer_deg_x10"]) / 10.0,
                    "wfl":   float(row["wfl"]),
                    "wfr":   float(row["wfr"]),
                    "wrl":   float(row["wrl"]),
                    "wrr":   float(row["wrr"]),
                    "brake": float(row["brake_bar"]),
                })
            except (ValueError, KeyError):
                continue
    return out


def _rel_seconds(records, key="t"):
    """Convert absolute ms timestamps to relative seconds from first sample."""
    if not records:
        return []
    t0 = records[0][key]
    return [(r[key] - t0) / 1000.0 for r in records]


def _save(fig, outdir, name):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, name)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f"  wrote {path}")


# ── Car-mode (OBD) figures ──────────────────────────────────────────────────

def plot_rpm(obd, outdir):
    pts = [(t, m["rpm"]) for t, m in zip(_rel_seconds(obd), obd)
           if m.get("rpm_v")]
    if not pts:
        print("  skip rpm_vs_time — no valid RPM samples"); return
    t, y = zip(*pts)
    fig, ax = plt.subplots()
    ax.plot(t, y, color="#c0392b")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Engine speed (rpm)")
    ax.set_title("Figure 14.3 — Engine RPM during test drive")
    _save(fig, outdir, "rpm_vs_time.png")


def plot_speed(obd, outdir):
    pts = [(t, m["spd"]) for t, m in zip(_rel_seconds(obd), obd)
           if m.get("spd_v")]
    if not pts:
        print("  skip speed_vs_time — no valid speed samples"); return
    t, y = zip(*pts)
    fig, ax = plt.subplots()
    ax.plot(t, y, color="#2471a3")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Vehicle speed (km/h)")
    ax.set_title("Figure 14.4 — Vehicle speed during test drive")
    _save(fig, outdir, "speed_vs_time.png")


def plot_coolant(obd, outdir):
    pts = [(t, m["clt"]) for t, m in zip(_rel_seconds(obd), obd)
           if m.get("clt_v")]
    if not pts:
        print("  skip coolant_warmup — no valid coolant samples"); return
    t, y = zip(*pts)
    fig, ax = plt.subplots()
    ax.plot(t, y, color="#d35400")
    ax.axhline(90, ls="--", color="gray", lw=1,
               label="thermostat plateau (~90 °C)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Coolant temperature (°C)")
    ax.set_title("Figure 14.5 — Engine coolant warm-up curve")
    ax.legend()
    _save(fig, outdir, "coolant_warmup.png")


def plot_multi_signal(obd, outdir):
    t = _rel_seconds(obd)
    if not t:
        print("  skip multi_signal — no OBD samples"); return
    rpm  = [m["rpm"]  if m.get("rpm_v")  else None for m in obd]
    thr  = [m["thr"]  if m.get("thr_v")  else None for m in obd]
    load = [m["load"] if m.get("load_v") else None for m in obd]
    fig, ax1 = plt.subplots()
    ax1.plot(t, rpm, color="#c0392b", label="RPM")
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Engine speed (rpm)",
                                               color="#c0392b")
    ax1.tick_params(axis="y", labelcolor="#c0392b")
    ax2 = ax1.twinx()
    ax2.plot(t, thr,  color="#27ae60", label="Throttle %")
    ax2.plot(t, load, color="#8e44ad", label="Load %")
    ax2.set_ylabel("Throttle / Load (%)")
    ax2.grid(False)
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="upper right")
    ax1.set_title("Figure 14.6 — RPM / throttle / load correlation")
    _save(fig, outdir, "multi_signal.png")


def plot_success_rate(obd, outdir):
    """Cumulative OBD counters req/ok/err/timeout vs time."""
    pts = [(t, m) for t, m in zip(_rel_seconds(obd), obd) if "req" in m]
    if not pts:
        print("  skip obd_success_rate — no counter fields"); return
    t   = [p[0] for p in pts]
    req = [p[1].get("req", 0) for p in pts]
    ok  = [p[1].get("ok", 0)  for p in pts]
    err = [p[1].get("err", 0) for p in pts]
    to  = [p[1].get("to", 0)  for p in pts]
    fig, ax = plt.subplots()
    ax.plot(t, req, color="#34495e", label="requests")
    ax.plot(t, ok,  color="#27ae60", label="ok")
    ax.plot(t, err, color="#e67e22", label="errors")
    ax.plot(t, to,  color="#c0392b", label="timeouts")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Cumulative count")
    pct = (ok[-1] / req[-1] * 100) if req and req[-1] else 0.0
    ax.set_title(f"Figure 14.10 — OBD success rate "
                 f"(final {pct:.1f}% ok)")
    ax.legend()
    _save(fig, outdir, "obd_success_rate.png")


# ── Bench / sniff figures (from decoded.csv) ────────────────────────────────

def plot_steering(dec, outdir):
    if not dec:
        print("  skip steering_vs_time — no decoded.csv"); return
    t = _rel_seconds(dec)
    y = [r["steer"] for r in dec]
    fig, ax = plt.subplots()
    ax.plot(t, y, color="#16a085")
    ax.axhline(0, ls="--", color="gray", lw=1, label="centre (0°)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Steering angle (°)")
    ax.set_title("Figure 14.7 — Steering angle (lock-to-lock test)")
    ax.legend()
    _save(fig, outdir, "steering_vs_time.png")


def plot_wheels(dec, outdir):
    if not dec:
        print("  skip wheel_speeds — no decoded.csv"); return
    t = _rel_seconds(dec)
    fig, ax = plt.subplots()
    for key, lbl, c in [("wfl", "Front-L", "#2980b9"),
                        ("wfr", "Front-R", "#27ae60"),
                        ("wrl", "Rear-L",  "#e67e22"),
                        ("wrr", "Rear-R",  "#c0392b")]:
        ax.plot(t, [r[key] for r in dec], label=lbl, color=c)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Wheel speed (km/h)")
    ax.set_title("Figure 14.8 — Per-wheel speed (turn shows differential)")
    ax.legend(ncol=4, fontsize=9)
    _save(fig, outdir, "wheel_speeds.png")


def plot_brake(dec, outdir):
    if not dec:
        print("  skip brake_pressure — no decoded.csv"); return
    t = _rel_seconds(dec)
    y = [r["brake"] for r in dec]
    fig, ax = plt.subplots()
    ax.plot(t, y, color="#7f3a17")
    ax.fill_between(t, y, alpha=0.2, color="#7f3a17")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Brake pressure (bar)")
    ax.set_title("Figure 14.9 — Brake pressure during pedal application")
    _save(fig, outdir, "brake_pressure.png")


# ── Demo data generator (no hardware / no logs needed) ──────────────────────

def make_demo(outdir):
    """Synthesise a plausible session so the plotting path can be proven."""
    import math, random
    sess = os.path.join(outdir, "demo_session")
    os.makedirs(sess, exist_ok=True)

    # raw.jsonl with OBD lines: idle -> rev -> cruise, coolant warming up
    req = ok = err = to = 0
    with open(os.path.join(sess, "raw.jsonl"), "w") as fh:
        for i in range(240):                       # 240 * 0.5 s = 120 s
            t = i * 500
            phase = i / 240.0
            rpm  = 850 + 2500 * max(0, math.sin(phase * 6.28 * 3)) \
                       + random.gauss(0, 40)
            spd  = max(0, 60 * phase + 10 * math.sin(phase * 6) + random.gauss(0, 1))
            thr  = min(100, max(8, rpm / 40 + random.gauss(0, 2)))
            load = min(100, max(15, thr * 0.8 + random.gauss(0, 3)))
            clt  = min(91, 25 + 70 * (1 - math.exp(-phase * 4)))
            intk = 30 + random.gauss(0, 1)
            req += 7; ok += 6 + random.randint(0, 1); to += random.randint(0, 1)
            err = req - ok - to
            fh.write(json.dumps({
                "t": t, "type": "obd",
                "rpm": round(rpm), "spd": round(spd), "thr": round(thr, 1),
                "load": round(load, 1), "clt": round(clt), "intake": round(intk),
                "rpm_v": 1, "spd_v": 1, "thr_v": 1, "load_v": 1,
                "clt_v": 1, "int_v": 1,
                "req": req, "ok": ok, "err": err, "to": to,
            }) + "\n")

    # decoded.csv: steering lock-to-lock + wheel differential in a turn
    with open(os.path.join(sess, "decoded.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_ms", "rpm", "steer_deg_x10",
                    "wfl", "wfr", "wrl", "wrr", "brake_bar", "valid_mask"])
        for i in range(200):
            t = i * 100
            ph = i / 200.0
            steer = 450 * math.sin(ph * 6.28)          # ±45° lock-to-lock
            base  = 40 + 20 * math.sin(ph * 3)
            turn  = steer / 100.0                       # outer wheels faster
            brake = max(0, 30 * math.sin(ph * 12)) if 0.4 < ph < 0.5 else 0
            w.writerow([t, round(900 + base * 30),
                        round(steer * 10),
                        round(base + turn), round(base - turn),
                        round(base + turn * 0.8), round(base - turn * 0.8),
                        round(brake), 15])
    print(f"  demo session written to {sess}")
    return sess


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Generate thesis validation plots")
    ap.add_argument("session", nargs="?", help="path to session_* directory")
    ap.add_argument("--outdir", default="figures", help="output directory")
    ap.add_argument("--demo", action="store_true",
                    help="synthesise demo data and plot it")
    args = ap.parse_args()

    if args.demo:
        args.session = make_demo(args.outdir)
    if not args.session:
        ap.error("provide a session directory or use --demo")

    print(f"Session : {args.session}")
    print(f"Figures : {args.outdir}/")

    obd = load_obd(args.session)
    dec = load_decoded(args.session)
    print(f"Loaded  : {len(obd)} OBD records, {len(dec)} decoded rows\n")

    # Car-mode figures
    plot_rpm(obd, args.outdir)
    plot_speed(obd, args.outdir)
    plot_coolant(obd, args.outdir)
    plot_multi_signal(obd, args.outdir)
    plot_success_rate(obd, args.outdir)
    # Bench / sniff figures
    plot_steering(dec, args.outdir)
    plot_wheels(dec, args.outdir)
    plot_brake(dec, args.outdir)

    print("\nDone.")


if __name__ == "__main__":
    main()
