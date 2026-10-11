// abs_config.h — every tunable in one place.
//
// Thresholds below are the values the car actually ran with. They are
// separated from the logic on purpose: on a test day you retune this
// file, rebuild, and reflash. You never edit the state machine to chase
// a handling problem.

#pragma once

#include <stdint.h>

namespace absfw {

// ── Wheel speed capture ────────────────────────────────────────────────
// 24-tooth trigger ring, 0.45 m rolling radius → 59 mm of arc per tooth.
constexpr uint32_t kTriggerTeeth      = 24;
constexpr float    kRollingRadiusM    = 0.45f;

// Physical bounds. Anything outside these is sensor trouble, not speed.
constexpr uint32_t kMinPeriodUs       = 150;    // ~240 km/h ceiling
constexpr uint32_t kMaxPeriodUs       = 500000; // < 2 km/h, effectively stopped

// A channel with no edge for this long is stale.
constexpr uint32_t kStaleAfterUs      = 50000;

// Edge-to-edge jitter above this multiple of the median marks NOISY.
constexpr float    kJitterRatio       = 3.0f;

// Equivalent-speed ceiling used to reject an implausible channel.
constexpr float    kMaxSpeedMs        = 38.9f;  // 140 km/h

// ── Slip estimation ────────────────────────────────────────────────────
// λ is a fixed-point u8 scaled by 255 so it fits a CAN byte losslessly.
constexpr uint8_t  kLambdaScale       = 255;

// Below this reference speed slip is numerically meaningless — the
// denominator swamps the difference — so we stop modulating and let the
// driver have the car.
constexpr float    kRefSpeedFloorMs   = 1.5f;

// v_ref quality gates. At least kMinPlausibleWheels must be healthy,
// and the spread between them must stay under kMaxSpreadRatio.
constexpr uint8_t  kMinPlausibleWheels = 2;
constexpr float    kMaxSpreadRatio     = 0.35f;

// ── ABS state machine ──────────────────────────────────────────────────
// Asymmetric on purpose. A symmetric pair chatters: the wheel recovers a
// little, we re-apply, it re-locks, and the pressure cycles faster than
// the hydraulics can follow.
constexpr float    kSlipEnter         = 0.15f;  // → RELEASE
constexpr float    kSlipExit          = 0.10f;  // → APPLY resumes ramping
constexpr float    kSlipSettled       = 0.06f;  // → event over, back to ARMED

// Minimum dwell times. These are what make the loop stable — without them
// the machine tracks sensor noise instead of tyre behaviour.
constexpr uint32_t kReleaseDwellMs    = 40;
constexpr uint32_t kHoldDwellMs       = 10;
constexpr uint32_t kSettleMs          = 150;

// ── Pressure control ───────────────────────────────────────────────────
// APPLY is a ramp, never a step: one motor pulse must not be able to
// jump the whole grip peak.
constexpr uint8_t  kApplyRampPctPerTick = 4;   // % of full pressure / cycle
constexpr uint8_t  kReleaseStepPct      = 30;  // dump hard, that's the point

// Closed loop on measured line pressure, not on motor command.
struct PidGains {
    float kp;
    float ki;
    float kd;
};
constexpr PidGains kPressurePid = {2.2f, 0.35f, 0.08f};

// ── Faults ─────────────────────────────────────────────────────────────
constexpr uint32_t kLoopOverrunUs    = 2500;
constexpr uint8_t  kOverrunTripCount = 3;      // within 1 s
constexpr float    kMotorOvercurrentA = 12.0f;
constexpr uint32_t kWatchdogMs       = 200;

}  // namespace absfw
