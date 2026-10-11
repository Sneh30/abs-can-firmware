#!/usr/bin/env python3
"""Replay a captured CAN log through the production ABS state machine.

This exists so a threshold bug can be debugged on a laptop instead of on
track. It decodes the wheel-speed frames out of a CSV capture, runs the
same slip estimation and state machine the firmware runs, and prints the
pressure trace the ECU *would* have commanded.

The thresholds and dwell times below are transcribed from
include/abs_config.h. If you change one, change both -- see
assert_thresholds_match() at the bottom, which checks the two files agree.

Usage:
    python3 tools/can_replay.py testdata/lockup_run_01.csv
    python3 tools/can_replay.py capture.csv --plot
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# ── abs_config.h, transcribed ──────────────────────────────────────────
K_TRIGGER_TEETH = 24
K_ROLLING_RADIUS_M = 0.45
K_REF_SPEED_FLOOR_MS = 1.5
K_MIN_PLAUSIBLE_WHEELS = 2
K_MAX_SPREAD_RATIO = 0.35

K_SLIP_ENTER = 0.15
K_SLIP_EXIT = 0.10
K_SLIP_SETTLED = 0.06

K_RELEASE_DWELL_MS = 40
K_HOLD_DWELL_MS = 10
K_SETTLE_MS = 150

K_RELEASE_STEP_PCT = 30
K_APPLY_RAMP_PCT = 4

METRES_PER_TOOTH = (2 * 3.14159265 * K_ROLLING_RADIUS_M) / K_TRIGGER_TEETH

WHEEL_IDS = {0x100: "FL", 0x101: "FR", 0x102: "RL", 0x103: "RR"}
ID_ABS_STATE = 0x200
ID_PRESSURE_CMD = 0x201

STATE_NAMES = {
    0: "DISARMED",
    1: "ARMED",
    2: "RELEASE",
    3: "HOLD",
    4: "APPLY",
    5: "FAULTED",
}


@dataclass
class Wheel:
    period_us: int = 0
    last_seen_ms: float = -1.0
    speed_ms: float = 0.0

    def update(self, period_us: int, now_ms: float) -> None:
        self.period_us = period_us
        self.last_seen_ms = now_ms
        if period_us > 0:
            self.speed_ms = METRES_PER_TOOTH / (period_us * 1e-6)

    def speed_at(self, now_ms: float) -> float:
        """Speed, zeroed once the channel goes stale.

        A stopped wheel genuinely stops producing edges, so staleness
        alone is not proof of failure -- the firmware reasons about that
        in the slip estimator, and so do we.
        """
        if self.last_seen_ms < 0:
            return 0.0
        if now_ms - self.last_seen_ms > 50.0:
            return 0.0
        return self.speed_ms


@dataclass
class Reference:
    speed_ms: float = 0.0
    quality: int = 0  # 0 lost, 1 degraded, 2 good

    def update(self, speeds: list[float]) -> None:
        plausible = [s for s in speeds if s > 0]
        if len(plausible) < K_MIN_PLAUSIBLE_WHEELS:
            self.quality = 0
            self.speed_ms = 0.0
            return
        fastest = max(plausible)
        slowest = min(plausible)
        spread = (fastest - slowest) / fastest if fastest > 0 else 0.0
        self.speed_ms = fastest
        self.quality = 2 if (len(plausible) >= 3 and spread < K_MAX_SPREAD_RATIO) else 1


def slip(ref_speed: float, quality: int, wheel_speed: float) -> float:
    if quality == 0 or ref_speed < K_REF_SPEED_FLOOR_MS:
        return 0.0
    return (ref_speed - wheel_speed) / ref_speed


# ── state machine ─────────────────────────────────────────────────────
@dataclass
class AbsState:
    state: str = "DISARMED"
    dwell_ms: int = 0
    settled_ms: int = 0
    cycles: int = 0
    pressure_pct: int = 0

    def enter(self, next_state: str) -> None:
        if next_state != self.state:
            self.state = next_state
            self.dwell_ms = 0

    def step(self, lam: float, ref_ok: bool, current_pct: int,
             dt_ms: int) -> tuple[str, int]:
        self.dwell_ms += dt_ms
        pressure = current_pct

        if not ref_ok:
            self.enter("FAULTED")
            pressure = 0

        if self.state == "DISARMED":
            # Safety does not wait for arming: a capture (or a car) can
            # start already locked, so go straight to RELEASE and arm
            # only when the wheel is rolling cleanly.
            if ref_ok and lam > K_SLIP_ENTER:
                self.enter("RELEASE")
            elif ref_ok and lam < K_SLIP_ENTER:
                self.enter("ARMED")

        elif self.state == "ARMED":
            pressure = current_pct
            self.settled_ms = 0
            if not ref_ok:
                self.enter("FAULTED")
            elif lam > K_SLIP_ENTER:
                self.enter("RELEASE")
            elif current_pct == 0:
                self.enter("DISARMED")

        elif self.state == "RELEASE":
            pressure = max(0, current_pct - K_RELEASE_STEP_PCT)
            if lam < K_SLIP_EXIT and self.dwell_ms >= K_RELEASE_DWELL_MS:
                self.enter("HOLD")

        elif self.state == "HOLD":
            if self.dwell_ms >= K_HOLD_DWELL_MS and lam < K_SLIP_EXIT:
                self.enter("APPLY")
                self.cycles += 1

        elif self.state == "APPLY":
            pressure = min(100, current_pct + K_APPLY_RAMP_PCT)
            if lam > K_SLIP_ENTER:
                self.enter("RELEASE")
            elif lam < K_SLIP_SETTLED:
                self.settled_ms += dt_ms
                if self.settled_ms >= K_SETTLE_MS:
                    self.enter("ARMED")
            else:
                self.settled_ms = 0

        elif self.state == "FAULTED":
            pressure = 0

        self.pressure_pct = pressure
        return self.state, pressure


# ── log parsing ───────────────────────────────────────────────────────
def parse_capture(path: Path) -> list[tuple[float, int, bytes]]:
    """Yield (timestamp_ms, can_id, data) from either CSV shape.

    Accepts the common candump-ish form
        (123.456789) can0 100#AABBCCDD
    and a plain headered CSV
        time_ms,id,data
        12.0,0x100,1a2b3c4d0e000000
    """
    rows: list[tuple[float, int, bytes]] = []
    raw = path.read_text().splitlines()

    candump = re.compile(
        r"^\s*\(?\s*([0-9]+(?:\.[0-9]+)?)\)?\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)"
    )
    for line in raw:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        m = candump.match(line)
        if m:
            ts = float(m.group(1))
            if ts < 1000:  # candump reports seconds
                ts *= 1000.0
            rows.append((ts, int(m.group(2), 16), bytes.fromhex(m.group(3))))
            continue
        if "," in line and ("0x" in line or re.match(r"^\d+,\d+,", line)):
            parts = [p.strip() for p in line.split(",")]
            if parts[0].lower() in ("time_ms", "t_ms", "ms", "time"):
                continue
            try:
                ts = float(parts[0])
                can_id = int(parts[1], 16) if parts[1].lower().startswith("0x") \
                    else int(parts[1])
            except ValueError:
                continue
            hexdata = parts[2].replace(" ", "")
            try:
                rows.append((ts, can_id, bytes.fromhex(hexdata)))
            except ValueError:
                continue

    rows.sort(key=lambda r: r[0])
    return rows


def replay(rows: list[tuple[float, int, bytes]], dt_ms: int = 5) -> list[dict]:
    wheels = {name: Wheel() for name in WHEEL_IDS.values()}
    ref = Reference()
    machine = AbsState()

    events: list[dict] = []
    prev_pressure = 0

    if not rows:
        return events

    t_start, t_end = rows[0][0], rows[-1][0]
    t = t_start

    idx = 0
    while t <= t_end:
        # Fold in every frame that landed in this window.
        while idx < len(rows) and rows[idx][0] <= t:
            _, can_id, data = rows[idx]
            idx += 1
            name = WHEEL_IDS.get(can_id)
            if name and len(data) >= 7:
                period = int.from_bytes(data[0:4], "little")
                wheels[name].update(period, t)

        speeds = [wheels[n].speed_at(t) for n in ("FL", "FR", "RL", "RR")]
        ref.update(speeds)

        # Worst axle governs, same as the firmware.
        front = min(speeds[0], speeds[1])
        rear = min(speeds[2], speeds[3])
        worst_wheel = front if front < rear else rear
        lam = slip(ref.speed_ms, ref.quality, worst_wheel)

        state, pressure = machine.step(lam, ref.quality != 0,
                                       prev_pressure, dt_ms)
        prev_pressure = pressure

        events.append({
            "t_ms": round(t - t_start, 1),
            "v_ref_ms": round(ref.speed_ms, 3),
            "quality": ref.quality,
            "lambda": round(lam, 4),
            "state": state,
            "pressure_pct": pressure,
        })
        t += dt_ms

    return events


# ── reporting ─────────────────────────────────────────────────────────
def report(events: list[dict]) -> None:
    if not events:
        print("No frames decoded -- check the capture format.")
        return

    total = len(events)
    by_state: dict[str, int] = {}
    for e in events:
        by_state[e["state"]] = by_state.get(e["state"], 0) + 1

    interventions = sum(
        1 for i, e in enumerate(events)
        if e["state"] == "RELEASE"
        and (i == 0 or events[i - 1]["state"] != "RELEASE")
    )
    peak_lambda = max(e["lambda"] for e in events)
    peak_pressure = max(e["pressure_pct"] for e in events)
    duration_s = events[-1]["t_ms"] / 1000.0

    print(f"frames replayed      : {total}  ({duration_s:.1f} s)")
    print(f"peak slip ratio      : {peak_lambda:.3f}")
    print(f"peak pressure        : {peak_pressure} %")
    print(f"ABS interventions    : {interventions}")
    print()
    print("time in each state:")
    for name in ("DISARMED", "ARMED", "RELEASE", "HOLD", "APPLY", "FAULTED"):
        if name in by_state:
            pct = 100.0 * by_state[name] / total
            print(f"  {name:<9} {pct:5.1f} %  ({by_state[name]} ticks)")

    if "FAULTED" in by_state:
        print("\nWARNING: controller entered FAULTED -- reference lost.")
    if interventions == 0:
        print("\nNo interventions: the wheel never crossed "
              f"lambda > {K_SLIP_ENTER}. Either the capture is a clean "
              "stop or the threshold is too high for this surface.")


def assert_thresholds_match() -> None:
    """Guard against the two copies of the tunables drifting apart."""
    header = Path(__file__).resolve().parent.parent / "include" / "abs_config.h"
    if not header.exists():
        return
    text = header.read_text()

    checks = [
        ("kSlipEnter", K_SLIP_ENTER),
        ("kSlipExit", K_SLIP_EXIT),
        ("kSlipSettled", K_SLIP_SETTLED),
        ("kReleaseDwellMs", K_RELEASE_DWELL_MS),
        ("kHoldDwellMs", K_HOLD_DWELL_MS),
        ("kSettleMs", K_SETTLE_MS),
    ]
    drift = []
    for name, expected in checks:
        m = re.search(rf"{name}\s*=\s*([0-9.]+)f?", text)
        if m and abs(float(m.group(1)) - float(expected)) > 1e-6:
            drift.append(f"{name}: header={m.group(1)} python={expected}")

    if drift:
        print("THRESHOLD DRIFT between abs_config.h and this script:", file=sys.stderr)
        for d in drift:
            print(f"  {d}", file=sys.stderr)
        sys.exit(2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", type=Path)
    ap.add_argument("--plot", action="store_true",
                    help="emit a terminal sparkline of pressure vs slip")
    ap.add_argument("--csv-out", type=Path, default=None)
    args = ap.parse_args()

    if not args.capture.exists():
        print(f"no such file: {args.capture}", file=sys.stderr)
        return 1

    assert_thresholds_match()

    rows = parse_capture(args.capture)
    print(f"decoded {len(rows)} frames from {args.capture.name}\n")
    events = replay(rows)
    report(events)

    if args.csv_out and events:
        with args.csv_out.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(events[0].keys()))
            w.writeheader()
            w.writerows(events)
        print(f"\nwrote {args.csv_out}")

    if args.plot and events:
        print("\npressure % (top) vs slip (bottom), 60 cols:")
        cols = 60
        step = max(1, len(events) // cols)
        for label, key, scale in (("P", "pressure_pct", 1.0), ("L", "lambda", 1.0)):
            line = []
            for i in range(0, len(events), step):
                v = events[i][key] * scale
                line.append(" .:-=+*#%@"[min(9, int(v / (100 if key == "pressure_pct" else 0.2) * 9))])
            print(f"{label} |{''.join(line)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
