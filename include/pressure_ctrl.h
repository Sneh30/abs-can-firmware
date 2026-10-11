// pressure_ctrl.h — inner closed loop on measured line pressure.
//
// Runs at the same 200 Hz as the ABS machine but is deliberately a
// separate object: the state machine decides *what* pressure it wants,
// this decides how to get there. Keeping them apart means a tuning
// change on the motor side can never alter ABS behaviour.

#pragma once

#include <stdint.h>

#include "abs_config.h"
#include "can_ids.h"

namespace absfw {

class PressureCtrl {
   public:
    void begin();

    // One PID step toward `target_pct` given `measured_pct` from the
    // line transducer. Returns the duty to apply to the H-bridge,
    // signed by direction (positive = build, negative = dump).
    int step(uint8_t target_pct, uint8_t measured_pct, uint32_t dt_ms);

    void reset() { integrator_ = 0.0f; prev_err_ = 0.0f; }

    // Latched on overcurrent: we stop driving and let the ABS machine
    // find out through the pressure feedback going stale.
    void cut()   { cut_ = true; }
    bool cut() const { return cut_; }

   private:
    float integrator_ = 0.0f;
    float prev_err_   = 0.0f;
    bool  cut_        = false;
};

}  // namespace absfw
