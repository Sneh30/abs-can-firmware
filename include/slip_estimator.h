// slip_estimator.h — vehicle reference speed and per-axle slip ratio.
//
// The awkward part of ABS on a student car: there is no optical or GPS
// ground-speed sensor. We estimate v_ref as the fastest wheel, which is
// sound during braking (no wheel outruns the car unless it has left the
// ground) and unsound at any other time. So the estimate is valid exactly
// when ABS is armed, which is why VEHICLE_REF carries an explicit quality.

#pragma once

#include "abs_config.h"
#include "can_ids.h"
#include "wheel_speed.h"

namespace absfw {

struct AxleSlip {
    float lambda;       // signed: negative means this wheel is faster than ref
    uint8_t lambda_fp;  // saturating u8 × 255, for the wire
    bool locked;        // λ above kSlipEnter
};

class SlipEstimator {
   public:
    void begin() { ref_quality_ = kRefQualityLost; }

    // Reference speed in m/s, valid only while quality != Lost.
    float refSpeedMs() const { return ref_speed_ms_; }
    RefQuality quality() const { return ref_quality_; }
    RefSource  source()  const { return ref_source_; }

    // Front axle first. Computed from the two front / two rear channels.
    AxleSlip front(const WheelSpeed& fl, const WheelSpeed& fr) const;
    AxleSlip rear (const WheelSpeed& rl, const WheelSpeed& rr) const;

    // Refresh the reference from all four channels. `imu_speed_ms` is an
    // optional cross-check (see .cpp for when it is trusted).
    void update(const WheelSpeed& fl, const WheelSpeed& fr,
                const WheelSpeed& rl, const WheelSpeed& rr,
                float imu_speed_ms);

   private:
    AxleSlip slipOf(float wheel_ms) const;

    float       ref_speed_ms_ = 0.0f;
    RefQuality  ref_quality_  = kRefQualityLost;
    RefSource   ref_source_   = kRefSourceNone;
};

}  // namespace absfw
