// wheel_speed.h — capture, validity, and filtering of one hub channel.
//
// One instance per corner. The capture path is deliberately dumb: it
// latches a hardware counter and a timestamp and pushes to a ring
// buffer. All interpretation happens later in the 200 Hz control task.

#pragma once

#include <stdint.h>

#include "abs_config.h"

namespace absfw {

struct WheelSample {
    uint32_t period_us;   // time between the last two tooth edges
    uint16_t ticks;       // tooth counter since boot
    uint8_t  flags;       // kFlag*
    bool     fresh;       // true only for the cycle that consumed it
};

class WheelSpeed {
   public:
    // Called from the PCNT ISR. Must stay allocation-free and bounded.
    void onEdge(uint32_t period_us, uint32_t now_us);

    // Called from the 200 Hz task. Marks stale/impossible channels and
    // refreshes the derived speed. `now_us` is the control tick time.
    void evaluate(uint32_t now_us);

    // Metres/second, or 0.0f when the channel is not trustworthy.
    float speedMs() const { return speed_ms_; }

    uint8_t  flags()   const { return flags_; }
    bool     usable()  const { return (flags_ & kFlagValid) != 0; }
    uint32_t ageUs()   const { return age_us_; }
    uint32_t lastPeriodUs() const { return last_period_us_; }

   private:
    static constexpr int kRingSize = 8;

    uint32_t ring_[kRingSize] = {};
    int      head_ = 0;
    int      count_ = 0;

    uint32_t last_period_us_ = 0;
    uint32_t age_us_         = UINT32_MAX;
    uint16_t ticks_          = 0;
    uint8_t  flags_          = 0;
    float    speed_ms_       = 0.0f;

    float periodToSpeed(uint32_t period_us) const;
};

}  // namespace absfw
