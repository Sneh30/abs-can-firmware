// wheel_speed.cpp

#include "wheel_speed.h"

#include <cmath>

namespace absfw {

namespace {
// Circumference per tooth, in metres.
constexpr float kMetresPerTooth =
    (2.0f * 3.14159265f * kRollingRadiusM) / (float)kTriggerTeeth;
}

void WheelSpeed::onEdge(uint32_t period_us, uint32_t now_us) {
    (void)now_us;

    // Reject the physically impossible before it reaches the ring: a
    // 150 µs period at 24 teeth is ~240 km/h. Anything faster is ringing
    // on the sensor wire, not a wheel.
    if (period_us < kMinPeriodUs) {
        flags_ |= kFlagImplausible;
        return;
    }

    ring_[head_] = period_us;
    head_ = (head_ + 1) % kRingSize;
    if (count_ < kRingSize) ++count_;

    last_period_us_ = period_us;
    age_us_ = 0;
    ++ticks_;
    flags_ |= kFlagValid;
    flags_ &= ~static_cast<uint8_t>(kFlagStale | kFlagImplausible);
}

void WheelSpeed::evaluate(uint32_t now_us) {
    (void)now_us;

    // Staleness is measured, not assumed: a stopped wheel genuinely stops
    // generating edges, so we have to distinguish "locked" from "dead
    // sensor" using the ABS context rather than here.
    if (age_us_ > kStaleAfterUs) flags_ |= kFlagStale;

    speed_ms_ = periodToSpeed(last_period_us_);

    if (speed_ms_ > kMaxSpeedMs) {
        flags_ |= kFlagImplausible;
    }

    // Jitter check across the ring. A clean hall sensor on a turning hub
    // gives very consistent periods; a chattering one does not.
    if (count_ >= 4) {
        uint32_t lo = UINT32_MAX, hi = 0;
        for (int i = 0; i < count_; ++i) {
            const uint32_t p = ring_[i];
            if (p < lo) lo = p;
            if (p > hi) hi = p;
        }
        if (lo > 0 && (float)(hi - lo) / (float)lo > kJitterRatio) {
            flags_ |= kFlagNoisy;
        }
    }
}

float WheelSpeed::periodToSpeed(uint32_t period_us) const {
    if (period_us == 0 || period_us > kMaxPeriodUs) return 0.0f;
    // v = (metres per tooth) / (seconds per tooth)
    return kMetresPerTooth / ((float)period_us * 1e-6f);
}

}  // namespace absfw
