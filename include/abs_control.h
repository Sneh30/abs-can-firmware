// abs_control.h — the state machine.
//
// Asymmetric thresholds and minimum dwell times. Both are load-bearing:
// a symmetric pair chatters, and without dwell the machine tracks sensor
// noise instead of tyre behaviour.

#pragma once

#include <stdint.h>

#include "abs_config.h"
#include "can_ids.h"

namespace absfw {

enum class AbsState : uint8_t {
    kDisarmed = 0,  // not braking / below the reference floor
    kArmed    = 1,  // monitoring, pressure untouched
    kRelease  = 2,  // dumping pressure to stop the lockup
    kHold     = 3,  // valve shut, waiting for λ to fall
    kApply    = 4,  // ramping pressure back toward the grip peak
    kFaulted  = 5,  // released, driver has direct control
};

struct AbsOutput {
    AbsState  state;
    uint8_t   pressure_pct;   // commanded line pressure, 0-100
    uint8_t   direction;      // kPressure*
    uint16_t  cycles;         // modulation cycles in this event
    uint16_t  flags;          // kAbsFlag*
};

class AbsControl {
   public:
    void begin();
    void fault(uint16_t code, uint8_t severity);

    // `lambda` is the worst axle slip, `dt_ms` the control period.
    // `ref_ok` false forces a fault — modulating on a bad reference is
    // worse than not modulating at all. `ref_degraded` is advisory and
    // only reaches the wire, never the logic.
    AbsOutput step(float lambda, bool ref_ok, bool ref_degraded,
                   uint8_t current_pct, uint32_t dt_ms);

    AbsState state() const { return state_; }

   private:
    void enter(AbsState next);

    AbsState state_        = AbsState::kDisarmed;
    uint32_t dwell_ms_     = 0;
    uint32_t settled_ms_   = 0;
    uint16_t cycles_       = 0;
    uint16_t fault_flags_  = 0;
    uint8_t  pressure_pct_ = 0;
};

}  // namespace absfw
