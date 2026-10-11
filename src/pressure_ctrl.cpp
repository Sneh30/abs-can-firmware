// pressure_ctrl.cpp

#include "pressure_ctrl.h"

namespace absfw {

void PressureCtrl::begin() {
    integrator_ = 0.0f;
    prev_err_   = 0.0f;
    cut_        = false;
}

int PressureCtrl::step(uint8_t target_pct, uint8_t measured_pct,
                       uint32_t dt_ms) {
    if (cut_) return 0;

    const float dt = (float)dt_ms * 1e-3f;
    const float err = (float)target_pct - (float)measured_pct;

    // Anti-windup. The integrator is clamped and frozen when the
    // actuator is saturated, otherwise a long RELEASE phase leaves a
    // stored demand that spikes pressure the instant we re-apply.
    integrator_ += err * dt;
    if (integrator_ > 100.0f) integrator_ = 100.0f;
    if (integrator_ < -100.0f) integrator_ = -100.0f;

    const float deriv = (err - prev_err_) / (dt > 0.0f ? dt : 1.0f);
    prev_err_ = err;

    float out = kPressurePid.kp * err +
                kPressurePid.ki * integrator_ +
                kPressurePid.kd * deriv;

    if (out > 100.0f) out = 100.0f;
    if (out < -100.0f) out = -100.0f;

    // Deadband: below this the motor just heats and the lead screw
    // jitters without moving the cylinder.
    if (out > -3.0f && out < 3.0f) out = 0.0f;

    return (int)out;
}

}  // namespace absfw
