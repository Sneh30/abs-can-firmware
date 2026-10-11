// abs_control.cpp

#include "abs_control.h"

namespace absfw {

void AbsControl::begin() {
    state_        = AbsState::kDisarmed;
    dwell_ms_     = 0;
    settled_ms_   = 0;
    cycles_       = 0;
    fault_flags_  = 0;
    pressure_pct_ = 0;
}

void AbsControl::enter(AbsState next) {
    if (next != state_) {
        state_    = next;
        dwell_ms_ = 0;
    }
}

void AbsControl::fault(uint16_t code, uint8_t severity) {
    (void)code;
    if (severity >= 2) {
        enter(AbsState::kFaulted);
        pressure_pct_ = 0;  // release — the driver gets the pedal back
    }
}

AbsOutput AbsControl::step(float lambda, bool ref_ok, bool ref_degraded,
                           uint8_t current_pct, uint32_t dt_ms) {
    pressure_pct_ = current_pct;

    // Losing the reference is the one condition that always wins: ABS
    // would be modulating blind, so it steps aside and says so.
    if (!ref_ok) {
        enter(AbsState::kFaulted);
        pressure_pct_ = 0;
    }

    dwell_ms_ += dt_ms;

    switch (state_) {
        case AbsState::kDisarmed:
            // Safety does not wait for arming: the car can start with a
            // wheel already locked, so go straight to RELEASE and arm
            // only once the wheel is rolling cleanly.
            if (ref_ok && lambda > kSlipEnter) {
                enter(AbsState::kRelease);
                fault_flags_ |= kAbsFlagIntervene;
            } else if (ref_ok && lambda < kSlipEnter) {
                enter(AbsState::kArmed);
            }
            break;

        case AbsState::kArmed:
            pressure_pct_ = current_pct;  // driver's pressure, untouched
            settled_ms_   = 0;
            if (!ref_ok) {
                enter(AbsState::kFaulted);
            } else if (lambda > kSlipEnter) {
                enter(AbsState::kRelease);
            } else if (current_pct == 0) {
                enter(AbsState::kDisarmed);
            }
            break;

        case AbsState::kRelease:
            // Dump hard — that is the entire point of this state.
            pressure_pct_ =
                current_pct > kReleaseStepPct
                    ? (uint8_t)(current_pct - kReleaseStepPct)
                    : 0;
            fault_flags_ |= kAbsFlagIntervene;

            if (lambda < kSlipExit && dwell_ms_ >= kReleaseDwellMs) {
                enter(AbsState::kHold);
            }
            // Dwell is a floor, not a target: we may release for longer.
            break;

        case AbsState::kHold:
            // Valve shut. Wait for the wheel to speed back up.
            if (dwell_ms_ >= kHoldDwellMs && lambda < kSlipExit) {
                enter(AbsState::kApply);
                ++cycles_;  // one complete release/hold/apply cycle
            }
            break;

        case AbsState::kApply: {
            // Ramp, never step: one motor pulse must not be able to jump
            // the whole grip peak.
            const uint16_t ramped = (uint16_t)pressure_pct_ + kApplyRampPctPerTick;
            pressure_pct_ = ramped > 100 ? 100 : (uint8_t)ramped;

            if (lambda > kSlipEnter) {
                enter(AbsState::kRelease);  // it is locking again
            } else if (lambda < kSlipSettled) {
                settled_ms_ += dt_ms;
                if (settled_ms_ >= kSettleMs) {
                    enter(AbsState::kArmed);  // event over
                    fault_flags_ &= ~static_cast<uint16_t>(kAbsFlagIntervene);
                }
            } else {
                settled_ms_ = 0;
            }
            break;
        }

        case AbsState::kFaulted:
            pressure_pct_ = 0;
            break;
    }

    AbsOutput out{};
    out.state        = state_;
    out.pressure_pct = pressure_pct_;
    out.direction    = kPressureIdle;
    out.cycles       = cycles_;
    out.flags        = fault_flags_;

    switch (state_) {
        case AbsState::kRelease: out.direction = kPressureRelease; break;
        case AbsState::kHold:    out.direction = kPressureHold;    break;
        case AbsState::kApply:   out.direction = kPressureApply;   break;
        default: break;
    }
    if (ref_degraded) out.flags |= kAbsFlagRefDegrad;
    if (state_ == AbsState::kArmed || state_ == AbsState::kDisarmed)
        out.flags |= kAbsFlagArmed;

    return out;
}

}  // namespace absfw
