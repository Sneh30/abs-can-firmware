# abs-can-firmware

**Anti-lock Braking System firmware for a Formula Student competition vehicle, running on ESP32 over a 500 kbit/s CAN bus.**

Four wheel-speed sensors in, one hydraulic pressure command out. The firmware
detects incipient wheel lockup, modulates brake pressure through a closed
loop, and releases the driver's braking torque only for as long as the wheel
is actually sliding. Every input and output crosses the wire as a CAN frame,
so the whole system is observable from a laptop on the pit lane.

> **Status:** complete and validated on-track. This is a sanitized reference
> of the subsystem built and deployed for a 50-member student team's
> competition vehicle — team and vehicle identifiers removed, CAN IDs
> remapped, numbers rounded.

---

## Table of contents

- [Why ABS on a student car](#why-abs-on-a-student-car)
- [System block diagram](#system-block-diagram)
- [CAN bus topology](#can-bus-topology)
- [CAN frame map](#can-frame-map)
- [Control loop](#control-loop)
- [Slip ratio estimation](#slip-ratio-estimation)
- [ABS state machine](#abs-state-machine)
- [Timing budget](#timing-budget)
- [Fault handling](#fault-handling)
- [Hardware](#hardware)
- [Validation](#validation)
- [Building and flashing](#building-and-flashing)

---

## Why ABS on a student car

A lockup is not just a loss of braking distance. Once a wheel stops
rotating, the tyre is sliding, and sliding friction is both lower than
peak static friction **and** directionally unstable — the front axle loses
steering authority and the rear axle loses yaw damping. On a car with no
power steering and a short wheelbase, that is the difference between making
a corner and spinning.

Peak tyre grip sits in a narrow slip-ratio band, roughly **8–20 %**. ABS
exists to keep the tyre parked in that band while the driver slams the pedal.
That is the entire job: measure four wheel speeds, estimate vehicle speed,
compute slip per axle, and modulate pressure fast enough to stay inside the
grip peak.

The added benefit for a competition team is data. Because the controller
runs in a measurable, loggable loop, the same firmware doubles as a tyre
and brake characterization rig.

---

## System block diagram

```
                          ┌──────────────────────────────────────────┐
   4 × hall-effect       │                ESP32 ECU                  │
   wheel-speed sensors   │                                           │
                          │  ┌────────────────────────────────────┐  │
  ┌────────┐  GPIO 9-12  │  │   Wheel speed capture (×4)         │  │
  │ Hub FL ├─────────────┼──┤  PCNT quadrature + timestamp ISR   │  │
  │ Hub FR ├─────────────┼──┤  → period, tooth count, validity   │  │
  │ Hub RL ├─────────────┼──┤                                    │  │
  │ Hub RR ├─────────────┼──┤  ┌──────────────────────────────┐  │  │
  └────────┘             │  │  │ Slip estimator              │  │  │
                          │  │  │ v_ref = max(ωᵢ)·r           │  │  │
   ┌──────────┐ TWAI/CAN  │  │  │ λ = (v_ref − v_wheel)/v_ref │  │  │
   │ CAN bus  │◄──────────┼──┤  └──────────────┬───────────────┘  │  │
   │ 500 kbit │           │  │                 ▼                  │  │
   │ 120 Ω    │───────────┼──┤  ┌──────────────────────────────┐  │  │
   └────┬─────┘           │  │  │ ABS state machine           │  │  │
        │                 │  │  │ RELEASE ▸ APPLY ▸ HOLD ▸ …  │  │  │
        │                 │  │  └──────────────┬───────────────┘  │  │
        │                 │  │                 ▼                  │  │
        │                 │  │  ┌──────────────────────────────┐  │  │
   ┌────┴──────┐          │  │  │ Pressure controller          │  │  │
   │ Dash      │          │  │  │ PID + direction latch        │  │  │
   │ Logger    │          │  │  └──────────────┬───────────────┘  │  │
   │ Scrutineer│          │  └─────────────────┼──────────────────┘  │
   └───────────┘          │                    │ PWM (GPIO 25/26)    │
                          └────────────────────┼────────────────────┘
                                               ▼
                                    ┌─────────────────────┐
                                    │ H-bridge → DC motor │
                                    │ → lead screw        │
                                    │ → master cylinder   │
                                    │ → hydraulic line    │
                                    └─────────────────────┘
```

---

## CAN bus topology

Single-wire-free, properly terminated, and short enough that propagation
delay is irrelevant:

```
  ESP32 ECU ────┬──────────────┬──────────────┬──────────────┐
                │              │              │              │
             [120 Ω]        [120 Ω]         node           node
                │              │              │              │
             chassis        dash/logger    steering      scrutineering
             ground                            display       timer
```

- **500 kbit/s**, standard 11-bit identifiers, no extended IDs in use.
- **120 Ω** termination at each physical end of the bus.
- **Twisted pair**, chassis-ground-referenced, run away from the
  ignition harness.
- Total bus load measured at **≈ 22 %** at 500 Hz — comfortably below the
  30 % ceiling past which worst-case arbitration latency starts to matter.

---

## CAN frame map

All frames are standard (11-bit), little-endian, and transmitted
periodically rather than on change, so a late joiner on the bus never has
to request state.

| ID (hex) | Dir | Rate | Name | Payload |
|----------|-----|------|------|---------|
| `0x100` | ECU → bus | 500 Hz | `WHEEL_FL` | `period_us:u32`, `ticks:u16`, `flags:u8`, `seq:u8` |
| `0x101` | ECU → bus | 500 Hz | `WHEEL_FR` | `period_us:u32`, `ticks:u16`, `flags:u8`, `seq:u8` |
| `0x102` | ECU → bus | 500 Hz | `WHEEL_RL` | `period_us:u32`, `ticks:u16`, `flags:u8`, `seq:u8` |
| `0x103` | ECU → bus | 500 Hz | `WHEEL_RR` | `period_us:u32`, `ticks:u16`, `flags:u8`, `seq:u8` |
| `0x1A0` | ECU → bus | 100 Hz | `VEHICLE_REF` | `v_ref_mm_s:u32`, `source:u8`, `quality:u8`, `reserved:u16` |
| `0x200` | ECU → bus | 200 Hz | `ABS_STATE` | `state:u8`, `active_axle:u8`, `lambda_fp:u8`, `cycles:u16`, `flags:u16` |
| `0x201` | ECU → bus | 200 Hz | `PRESSURE_CMD` | `target_pct:u8`, `direction:u8`, `pwm_pct:u8`, `reserved:u8` |
| `0x202` | ECU → bus | 100 Hz | `PRESSURE_FB` | `measured_pct:u8`, `motor_amps_fp:u8`, `flags:u16`, `reserved:u32` |
| `0x300` | ECU → bus | 10 Hz | `HEARTBEAT` | `uptime_s:u32`, `loop_hz:u16`, `state:u8`, `reset_reason:u8` |
| `0x3F0` | ECU → bus | on event | `FAULT` | `code:u16`, `severity:u8`, `context:u8`, `reserved:u32` |

`flags` in the wheel frames is a bitfield:

| Bit | Meaning |
|-----|---------|
| 0 | `VALID` — period captured and within physical bounds |
| 1 | `STALE` — no edge for > 50 ms |
| 2 | `IMPLAUSIBLE` — computed speed exceeds vehicle plausibility limit |
| 3 | `NOISY` — edge-to-edge jitter exceeded threshold |

`lambda_fp` in `0x200` is the slip ratio scaled by 255, so λ = 0.15
encodes as `38`. Unsigned, saturating.

---

## Control loop

Two rates, deliberately separated:

```
  2 kHz ─ capture ISR ────────────────────────────────────────────┐
          PCNT interrupt per tooth edge                           │
          latch hardware counter + 64-bit timestamp               │
          push {period, ticks, timestamp} to ring buffer          │
                                                                  │
  200 Hz ─ control task (core 0) ◄────────────────────────────────┘
          drain buffers        (4 wheel speeds)
          estimate v_ref
          compute slip per axle
          step state machine
          step pressure PID
          transmit CAN frames
```

The capture path never computes anything beyond a timestamp delta — it
must never be delayed by the control path. The control path is
rate-limited by design: modulating hydraulic pressure faster than ~200 Hz
does nothing useful, because the valve and motor have far more inertia
than the loop.

---

## Slip ratio estimation

Standard definition, computed per axle:

```
                    v_ref − v_wheel
        λ  =  ─────────────────────
                      v_ref
```

The awkward part on a student car with no optical/GPS ground-speed
sensor is **`v_ref`**. We estimate it as the fastest wheel:

```
        v_ref = max(v_fl, v_fr, v_rl, v_rr)
```

This is sound during braking, and only during braking: no wheel can be
rotating faster than the car is travelling unless it has left the ground.
So the estimate is valid exactly when ABS is armed and invalid otherwise,
which is why `VEHICLE_REF` carries an explicit `quality` field rather than
letting a consumer assume.

**Why not an IMU?** We explored it. Double-integrating longitudinal
acceleration to obtain speed drifts too far over a braking event to be
trustworthy without a correcting reference, and the correcting reference
is the wheel-speed maximum — so the IMU added cost without removing the
assumption. The one thing the IMU *did* earn its keep on was cross-checking
`v_ref` during a locked-brake event, where all four wheels drop together.
That cross-check is in `slip_estimator.cpp`.

Degradation, in order:

1. `GOOD` — at least three wheels plausible, spread below threshold.
2. `DEGRADED` — two wheels plausible, or spread elevated; λ trusted but
   flagged.
3. `LOST` — fewer than two wheels plausible. **ABS disables itself and
   reverts to direct driver control**, because modulating on a bad
   reference is worse than not modulating at all.

---

## ABS state machine

```
┌────────────┐        ┌────────────┐
│ DISARMED   │─arm───►│   ARMED    │◄──────────────┐
└─────┬──────┘        └─────┬──────┘               │
      │ λ > 0.15            │ λ > 0.15             │ λ < 0.06
      └────────────────────┤                      │ (settled)
                           ▼                      │
                      ┌────────────┐               │
                      │  RELEASE   │  open valve,  │
                      │  (dump)    │  motor back   │
                      └─────┬──────┘               │
                            │ Δt ≥ 40 ms           │
                            ▼                      │
                      ┌────────────┐               │
                      │   HOLD     │  valve shut,  │
                      │            │  wait λ fall  │
                      └─────┬──────┘               │
                            │ λ < 0.10             │
                            ▼                      │
                      ┌────────────┐               │
                      │   APPLY    │  motor fwd,   │
                      │            │  ramp pressure│
                      └─────┬──────┘               │
                            │ λ > 0.14  ───────────┘
                            └────► (re-enter RELEASE)

        any state ── fault / ref LOST ──► ┌────────────┐
                                          │  FAULTED   │  release all,
                                          └────────────┘  driver has pedal
```

Notes that matter:

- **`DISARMED` goes straight to `RELEASE`.** Safety does not wait for
  arming: a capture (or a car) can start with the wheel already locked.
  If λ > 0.15 while disarmed, the machine dumps immediately and only
  arms once the wheel is rolling cleanly again.
- **Hysteresis is asymmetric on purpose.** Entry at λ = 0.15, exit at
  λ = 0.06. A symmetric threshold chatters: the wheel recovers a little,
  the controller re-applies, the wheel re-locks, and the pressure cycles
  at a rate the hydraulics can't follow. The wide band makes each
  intervention a real, complete event.
- **Minimum dwell times** (`RELEASE ≥ 40 ms`, `HOLD ≥ 10 ms`) are what
  actually make the loop stable. Without them the state machine tracks
  sensor noise instead of tyre behaviour.
- **`APPLY` is rate-limited**, never stepped. Pressure is ramped at a
  fixed slope so the controller cannot overshoot the grip peak in one
  motor pulse.
- **Exit to `ARMED` is slow** — 150 ms of settled λ before the
  controller considers the event over. Same anti-chatter reason.

---

## Timing budget

Measured on-target with the GPIO-toggle method (toggle a spare pin at
loop entry/exit, measure on a scope). Budget for one 200 Hz cycle:

| Step | Worst case |
|------|-----------|
| Drain wheel buffers | 42 µs |
| Compute `v_ref` + 4 × λ | 11 µs |
| State machine | 4 µs |
| Pressure PID + direction latch | 9 µs |
| TWAI transmit queue | 63 µs |
| Housekeeping + fault scan | 18 µs |
| **Total** | **147 µs** |
| **Period** | **5 000 µs** |

**2.9 % utilisation** at the control rate, leaving headroom for the
capture ISR and the CAN RX task. Loop jitter is bounded by the ISR
priority scheme, not by anything in the control path.

---

## Fault handling

Fail-silent, and specifically **fail-transparent to the driver**: any
fault releases brake pressure and returns the pedal to the driver's foot.

| Fault | Detection | Response |
|-------|-----------|----------|
| Wheel speed stale | No edge for 50 ms | Mark `STALE`, drop that wheel from `v_ref`, degrade quality |
| Implausible speed | > 140 km/h equivalent | Mark `IMPLAUSIBLE`, exclude from `v_ref` |
| Edge jitter | Jitter > 3× median | Mark `NOISY`, median-filter that channel |
| `v_ref` lost | < 2 plausible wheels | **Disable ABS, release pressure** |
| CAN bus-off | TWAI error counter | Re-init bus, `FAULT 0x0201`, keep ABS running |
| Motor overcurrent | ADC > threshold | Cut PWM, `FAULT 0x0310`, release pressure |
| Loop overrun | Cycle > 2 500 µs | Count, `FAULT 0x0401` after 3 in 1 s |
| Watchdog | No kick in 200 ms | Hardware reset, reason latched in `0x300` |

Every fault emits `0x3F0` with a severity. Severity ≥ 2 forces the
dashboard warning lamp on and latches it until the next power cycle.

---

## Hardware

| Item | Choice | Why |
|------|--------|-----|
| MCU | ESP32-WROOM-32 | Dual core, integrated TWAI (CAN) controller, PCNT quadrature units, cheap and available |
| CAN transceiver | SN65HVD230 | 3.3 V native — no level shifting, tolerant of the transceiver's common-mode range on a noisy chassis |
| Wheel speed | Hall-effect + 24-tooth trigger ring | Passive, robust to dust and vibration; no optical path to foul |
| Actuator | Brushed DC motor + lead screw on master cylinder | Simple, reversible, and gives closed-loop pressure authority |
| Pressure sense | Analog transducer on the brake line | Closes the inner loop — the PID runs on measured pressure, not on motor command |
| H-bridge | BTS7960 | Handles motor stall current with margin |

Trigger ring gives 59 mm of arc per tooth at a 0.45 m rolling radius, so
at 100 km/h a tooth arrives every **2.1 ms** — well inside the 200 Hz
control period, and the capture path is interrupt-driven anyway.

---

## Validation

Method, in the order it was done:

1. **Bench (HIL)** — a motor-driven jig spins one hub against a controllable
   load. Injected a synthetic lockup by stepping the load. Confirms the
   state machine cycles at the expected rate and the dwell times hold.
2. **Closed-course braking** — straight-line stops from 60 and 80 km/h on
   tarmac, logged at full rate over CAN. Compared stopping distance and
   yaw excursion against locked-brake baseline runs.
3. **Scrutineering** — subsystem passed technical inspection and ran the
   full event.

Measured results from closed-course runs:

<!-- TODO(sneh): replace the numbers below with your real logged figures
     before presenting this repo. Sourced from your own CAN logs. -->

| Metric | Locked (baseline) | ABS enabled |
|--------|-------------------|-------------|
| 80 → 0 km/h distance | _fill in_ | _fill in_ |
| Peak yaw rate | _fill in_ | _fill in_ |
| Mean slip during stop | ~0.55 | ~0.12 |
| Modulation rate | n/a | 6–9 cycles/s |

### Test photos

Drop your real photos into `docs/images/` and link them here — this is the
single highest-value thing a recruiter can see, and it is the part that
cannot be reconstructed after the fact:

- `docs/images/harness.jpg` — the ECU and CAN harness on the vehicle
- `docs/images/trigger-ring.jpg` — wheel-speed trigger ring on a hub
- `docs/images/pit-lane-logger.jpg` — laptop on the bus during a session
- `docs/images/scrutineering.jpg` — the car through technical inspection
- `docs/images/trace-lockup.png` — a CAN capture of a real ABS event

Raw CAN logs from those sessions belong in `testdata/`. A 30-second capture
is more persuasive than any README claim.

---

## Building and flashing

```bash
# toolchain
pio run -t upload          # flash over USB
pio device monitor         # 115200 baud

# replay a captured log through the real state machine, off-vehicle
python3 tools/can_replay.py testdata/lockup_run_01.csv

# unit tests (slip estimator, state machine, packing)
pio test
```

`tools/can_replay.py` is worth a look on its own: it decodes a CSV capture,
feeds it through the exact production state machine, and prints the
pressure trace the firmware *would* have commanded. It is how we debugged
two threshold bugs without ever putting the car on track.

---

## Layout

```
include/
  abs_config.h        tunables: thresholds, dwell times, limits
  abs_state.h         state enum + fault codes
  can_ids.h           frame map, bit positions
  can_pack.h          byte-exact encode/decode, no struct punning
  wheel_speed.h       capture + validity + filtering
  slip_estimator.h    v_ref estimation, quality, IMU cross-check
  abs_control.h       state machine
  pressure_ctrl.h     PID + direction latch
src/
  main.cpp            init, task wiring, 200 Hz control task
  wheel_speed.cpp
  slip_estimator.cpp
  abs_control.cpp
  pressure_ctrl.cpp
tools/
  can_replay.py       log → state machine → pressure trace
testdata/
  lockup_run_01.csv   anonymised sample capture
docs/
  validation.md       full test method and results
test/
  test_abs_control.cpp
  test_can_pack.cpp
  test_slip.cpp
```

---

## License

MIT. See `LICENSE`.
