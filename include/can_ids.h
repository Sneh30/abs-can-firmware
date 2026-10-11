// can_ids.h — the frame map.
//
// Standard 11-bit IDs only. Every periodic frame is transmitted at a
// fixed rate rather than on change, so a node that joins the bus late
// never has to request state.

#pragma once

#include <stdint.h>

namespace absfw {

// Wheel speed, one frame per corner, 500 Hz.
constexpr uint16_t kIdWheelFL = 0x100;
constexpr uint16_t kIdWheelFR = 0x101;
constexpr uint16_t kIdWheelRL = 0x102;
constexpr uint16_t kIdWheelRR = 0x103;

// Derived vehicle reference speed, 100 Hz.
constexpr uint16_t kIdVehicleRef = 0x1A0;

// Controller state, 200 Hz.
constexpr uint16_t kIdAbsState   = 0x200;

// Commanded brake pressure, 200 Hz.
constexpr uint16_t kIdPressureCmd = 0x201;

// Measured line pressure, 100 Hz.
constexpr uint16_t kIdPressureFb  = 0x202;

// Liveness, 10 Hz.
constexpr uint16_t kIdHeartbeat   = 0x300;

// Faults, on event.
constexpr uint16_t kIdFault       = 0x3F0;

// ── Frame payload layouts ──────────────────────────────────────────────
// Little-endian throughout. Byte offsets, not bitfields, so the encoder
// is explicit and the layout cannot silently change with a compiler flag.

// 0x100-0x103  WHEEL_*   (8 bytes)
//   [0..3]  period_us    u32  time between last two tooth edges
//   [4..5]  ticks        u16  tooth counter since boot
//   [6]     flags        u8   see WheelFlags
//   [7]     seq          u8   rolling counter, wraps at 255

// WheelFlags — bit positions inside byte 6.
constexpr uint8_t kFlagValid      = 1u << 0;  // captured, in physical bounds
constexpr uint8_t kFlagStale      = 1u << 1;  // no edge for > kStaleAfterUs
constexpr uint8_t kFlagImplausible= 1u << 2;  // exceeds kMaxSpeedMs
constexpr uint8_t kFlagNoisy      = 1u << 3;  // jitter above threshold

// 0x1A0  VEHICLE_REF   (8 bytes)
//   [0..3]  v_ref_mm_s   u32  millimetres/second, avoids float on the wire
//   [4]     source       u8   RefSource
//   [5]     quality      u8   RefQuality
//   [6..7]  reserved     u16

enum RefSource : uint8_t {
    kRefSourceNone     = 0,
    kRefSourceMaxWheel = 1,  // the normal braking-time path
    kRefSourceImuCross  = 2, // IMU agreed, used during full lockup
};

enum RefQuality : uint8_t {
    kRefQualityLost     = 0,  // fewer than 2 plausible wheels — ABS disables
    kRefQualityDegraded = 1,  // 2 plausible wheels, or spread elevated
    kRefQualityGood     = 2,  // 3+ plausible wheels, spread nominal
};

// 0x200  ABS_STATE    (8 bytes)
//   [0]     state          u8   AbsState
//   [1]     active_axle    u8   0=front, 1=rear, 0xFF=none
//   [2]     lambda_fp      u8   slip × 255, saturating
//   [3..4]  cycles         u16  modulation cycles this event
//   [5..6]  flags          u16  see AbsFlags
//   [7]     reserved       u8

// AbsFlags — bit positions inside bytes 5..6 (little-endian u16).
constexpr uint16_t kAbsFlagArmed     = 1u << 0;
constexpr uint16_t kAbsFlagIntervene = 1u << 1;  // actively modulating
constexpr uint16_t kAbsFlagRefDegrad = 1u << 2;
constexpr uint16_t kAbsFlagDriverFb  = 1u << 3;  // pedal feedback active

// 0x201  PRESSURE_CMD  (8 bytes)
//   [0]     target_pct    u8
//   [1]     direction     u8   PressureDir
//   [2]     pwm_pct       u8
//   [3..7]  reserved      u8×5

enum PressureDir : uint8_t {
    kPressureIdle   = 0,
    kPressureApply  = 1,
    kPressureHold   = 2,
    kPressureRelease= 3,
};

// 0x300  HEARTBEAT    (8 bytes)
//   [0..3]  uptime_s      u32
//   [4..5]  loop_hz       u16  actual measured rate, not the nominal
//   [6]     state         u8
//   [7]     reset_reason  u8

// 0x3F0  FAULT        (8 bytes)
//   [0..1]  code          u16
//   [2]     severity      u8   1=notice, 2=warning(latch lamp), 3=fatal
//   [3]     context       u8
//   [4..7]  reserved      u32

// Fault codes.
constexpr uint16_t kFaultWheelStale      = 0x0101;
constexpr uint16_t kFaultWheelImplaus    = 0x0102;
constexpr uint16_t kFaultRefLost         = 0x0110;
constexpr uint16_t kFaultCanBusOff       = 0x0201;
constexpr uint16_t kFaultMotorOvercurrent= 0x0310;
constexpr uint16_t kFaultLoopOverrun     = 0x0401;
constexpr uint16_t kFaultPressureSensor  = 0x0501;

}  // namespace absfw
