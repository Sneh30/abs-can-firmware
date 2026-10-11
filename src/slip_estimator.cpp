// slip_estimator.cpp

#include "slip_estimator.h"

#include <cmath>

namespace absfw {

namespace {
uint8_t toFixed(float lambda) {
    if (lambda <= 0.0f) return 0;
    const float scaled = lambda * (float)kLambdaScale;
    return scaled >= 255.0f ? 255 : (uint8_t)(scaled + 0.5f);
}
}  // namespace

void SlipEstimator::update(const WheelSpeed& fl, const WheelSpeed& fr,
                           const WheelSpeed& rl, const WheelSpeed& rr,
                           float imu_speed_ms) {
    const WheelSpeed* all[4] = {&fl, &fr, &rl, &rr};

    // Count usable channels and find the fastest. A stale or implausible
    // channel contributes nothing — we would rather modulate on three
    // wheels than on a lie from one.
    uint8_t plausible = 0;
    float fastest = 0.0f;
    float slowest = 1e9f;

    for (const WheelSpeed* w : all) {
        if (!w->usable()) continue;
        ++plausible;
        const float v = w->speedMs();
        if (v > fastest) fastest = v;
        if (v < slowest) slowest = v;
    }

    // ── Quality gate ────────────────────────────────────────────────
    if (plausible < kMinPlausibleWheels) {
        ref_quality_ = kRefQualityLost;
        ref_source_  = kRefSourceNone;
        ref_speed_ms_ = 0.0f;
        return;
    }

    const float spread =
        fastest > 0.0f ? (fastest - slowest) / fastest : 0.0f;

    // ── Reference selection ─────────────────────────────────────────
    // Detect a genuine four-wheel lockup: all channels usable, spread
    // tiny, and nothing still spinning. If the wheels agree with each
    // other but disagree with the IMU, the IMU is right — they are all
    // sliding. The gate is one-directional, so the IMU can never push the
    // reference down and never manufacture slip out of nothing.
    ref_source_  = kRefSourceMaxWheel;
    ref_speed_ms_ = fastest;

    const bool allLocked = plausible == 4 && spread < 0.10f && fastest < 5.0f;

    if (allLocked && imu_speed_ms > fastest * 1.5f &&
        imu_speed_ms < kMaxSpeedMs) {
        ref_speed_ms_ = imu_speed_ms;
        ref_source_   = kRefSourceImuCross;
    }

    ref_quality_ = (plausible >= 3 && spread < kMaxSpreadRatio)
                       ? kRefQualityGood
                       : kRefQualityDegraded;
}

AxleSlip SlipEstimator::front(const WheelSpeed& fl, const WheelSpeed& fr) const {
    // Worst of the axle — the wheel that is about to lock governs.
    const WheelSpeed& w = (fl.speedMs() < fr.speedMs()) ? fl : fr;
    return slipOf(w.usable() ? w.speedMs() : 0.0f);
}

AxleSlip SlipEstimator::rear(const WheelSpeed& rl, const WheelSpeed& rr) const {
    const WheelSpeed& w = (rl.speedMs() < rr.speedMs()) ? rl : rr;
    return slipOf(w.usable() ? w.speedMs() : 0.0f);
}

AxleSlip SlipEstimator::slipOf(float wheel_ms) const {
    AxleSlip out{};

    // Below the floor, slip is numerically meaningless: the denominator
    // swamps the difference and a millimetre of noise reads as 100 %
    // slip. We simply declare no slip and let the driver have the car.
    if (ref_quality_ == kRefQualityLost || ref_speed_ms_ < kRefSpeedFloorMs) {
        out.lambda    = 0.0f;
        out.lambda_fp = 0;
        out.locked    = false;
        return out;
    }

    out.lambda = (ref_speed_ms_ - wheel_ms) / ref_speed_ms_;
    out.lambda_fp = toFixed(out.lambda > 0.0f ? out.lambda : 0.0f);
    out.locked = out.lambda > kSlipEnter;
    return out;
}

}  // namespace absfw
