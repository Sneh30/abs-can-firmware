// main.cpp — init, task wiring, and the 200 Hz control loop.
//
// Core 0 runs the control task and the TWAI driver. Core 1 is left free
// for logging and the dashboard, which must never be able to delay a
// control cycle.

#include <Arduino.h>
#include <driver/twai.h>
#include <driver/pcnt.h>

#include "abs_config.h"
#include "abs_control.h"
#include "can_ids.h"
#include "can_pack.h"
#include "pressure_ctrl.h"
#include "slip_estimator.h"
#include "wheel_speed.h"

using namespace absfw;

// ── Pin map ────────────────────────────────────────────────────────────
static constexpr pcnt_unit_t kPcntUnit[4] = {
    PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
static constexpr int kSpeedGpio[4] = {36, 39, 34, 35};

static constexpr int kPwmGpioFwd = 25;
static constexpr int kPwmGpioRev = 26;
static constexpr int kCanTxGpio  = 5;
static constexpr int kCanRxGpio  = 4;
static constexpr int kPressureAdc = 32;
static constexpr int kMotorAdc    = 33;
static constexpr int kLedGpio     = 2;

static constexpr uint32_t kControlPeriodUs = 5000;  // 200 Hz
static constexpr uint32_t kControlPeriodMs =
    kControlPeriodUs / 1000;

static WheelSpeed     g_wheels[4];
static SlipEstimator  g_slip;
static AbsControl     g_abs;
static PressureCtrl   g_pressure;

static volatile uint32_t g_periodUs[4] = {};
static volatile uint8_t  g_edgeFlag[4] = {};

// ── Capture ISR ────────────────────────────────────────────────────────
// Deliberately dumb. Latch a period, raise a flag, leave. Everything
// else happens in the control task, which is allowed to be late; this
// is not.
static void IRAM_ATTR onPcntHit(void* arg) {
    const int i = (int)(intptr_t)arg;
    uint16_t count = 0;
    pcnt_get_counter_value(kPcntUnit[i], &count);
    pcnt_clear_counter(kPcntUnit[i]);

    // period = time since previous edge, measured by a free-running
    // microsecond timer sampled here rather than in the task.
    static uint32_t lastUs[4] = {};
    const uint32_t now = micros();
    if (lastUs[i] != 0) g_periodUs[i] = now - lastUs[i];
    lastUs[i] = now;

    g_edgeFlag[i] = 1;
    (void)count;
}

static void initCapture() {
    for (int i = 0; i < 4; ++i) {
        pinMode(kSpeedGpio[i], INPUT);
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = kSpeedGpio[i];
        cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
        cfg.channel        = PCNT_CHANNEL_0;
        cfg.unit           = kPcntUnit[i];
        cfg.counter_hlim   = 32767;
        cfg.counter_llim   = 0;
        cfg.pos_mode       = PCNT_COUNT_INC;
        cfg.neg_mode       = PCNT_COUNT_DIS;
        cfg.counter_clear  = PCNT_COUNT_DIS;
        pcnt_unit_config(&cfg);
        pcnt_set_event_value(kPcntUnit[i], PCNT_EVT_THRES_0, 1);
        pcnt_event_enable(kPcntUnit[i], PCNT_EVT_THRES_0);
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add(kPcntUnit[i], onPcntHit, (void*)(intptr_t)i);
        pcnt_counter_pause(kPcntUnit[i]);
        pcnt_counter_clear(kPcntUnit[i]);
        pcnt_counter_resume(kPcntUnit[i]);
    }
}

static void initCan() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)kCanTxGpio, (gpio_num_t)kCanRxGpio, TWAI_MODE_NORMAL);
    g.tx_queue_len = 16;
    g.rx_queue_len = 32;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
}

static void send(uint16_t id, const uint8_t d[8]) {
    twai_message_t m = {};
    m.identifier = id;
    m.data_length_code = 8;
    memcpy(m.data, d, 8);
    twai_transmit(&m, 0);  // non-blocking: never stall the control loop
}

static void driveMotor(int duty) {
    if (duty == 0) {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        return;
    }
    if (duty > 0) {
        ledcWrite(0, (uint32_t)duty);
        ledcWrite(1, 0);
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, (uint32_t)(-duty));
    }
}

// ── Control task ───────────────────────────────────────────────────────
static void controlTask(void*) {
    uint32_t seq = 0;
    uint32_t lastCycleUs = micros();

    for (;;) {
        const uint32_t t0 = micros();

        // 1. Drain capture flags into the wheel objects.
        for (int i = 0; i < 4; ++i) {
            if (g_edgeFlag[i]) {
                g_edgeFlag[i] = 0;
                g_wheels[i].onEdge(g_periodUs[i], t0);
            }
            g_wheels[i].evaluate(t0);
        }

        // 2. Reference + slip.
        // IMU cross-check placeholder: 0.0f disables that branch unless
        // a real longitudinal accel estimate is wired in.
        g_slip.update(g_wheels[0], g_wheels[1], g_wheels[2], g_wheels[3],
                      0.0f);

        const bool refOk = g_slip.quality() != kRefQualityLost;
        const bool refDeg = g_slip.quality() == kRefQualityDegraded;

        AxleSlip front = g_slip.front(g_wheels[0], g_wheels[1]);
        AxleSlip rear  = g_slip.rear(g_wheels[2], g_wheels[3]);
        const float worst = front.lambda < rear.lambda ? front.lambda
                                                        : rear.lambda;
        const uint8_t activeAxle = !refOk ? 0xFF
                                  : (front.lambda < rear.lambda ? 0 : 1);

        // 3. Read back line pressure, then step the ABS machine.
        const uint8_t measured =
            (uint8_t)constrain(map(analogRead(kPressureAdc), 0, 4095,
                                   0, 100), 0, 100);
        const AbsOutput out =
            g_abs.step(worst, refOk, refDeg, measured, kControlPeriodMs);

        // 4. Inner loop, then actuate.
        const int duty = g_pressure.step(out.pressure_pct, measured,
                                         kControlPeriodMs);
        driveMotor(duty);

        // 5. Emit the frame map.
        uint8_t d[8];
        for (int i = 0; i < 4; ++i) {
            WheelFrame wf{};
            wf.period_us = g_wheels[i].lastPeriodUs();
            wf.ticks     = (uint16_t)seq;
            wf.flags     = g_wheels[i].flags();
            wf.seq       = (uint8_t)(seq & 0xFF);
            packWheel(d, wf);
            send((uint16_t)(kIdWheelFL + i), d);
        }

        VehicleRefFrame vf{};
        vf.v_ref_mm_s = (uint32_t)(g_slip.refSpeedMs() * 1000.0f);
        vf.source     = g_slip.source();
        vf.quality    = g_slip.quality();
        packVehicleRef(d, vf);
        send(kIdVehicleRef, d);

        AbsStateFrame sf{};
        sf.state       = (uint8_t)out.state;
        sf.active_axle = activeAxle;
        sf.lambda_fp   = (worst > 0.0f && worst <= 1.0f)
                             ? (uint8_t)(worst * 255.0f) : 0;
        sf.cycles      = out.cycles;
        sf.flags       = out.flags;
        packAbsState(d, sf);
        send(kIdAbsState, d);

        d[0] = out.pressure_pct;
        d[1] = out.direction;
        d[2] = (uint8_t)constrain(map(analogRead(kMotorAdc), 0, 4095,
                                      0, 100), 0, 100);
        d[3] = 0;
        memset(&d[4], 0, 4);
        send(kIdPressureCmd, d);

        digitalWrite(kLedGpio, out.state == AbsState::kRelease ? 1 : 0);

        // 6. Overrun accounting.
        uint32_t elapsed = micros() - t0;
        if (elapsed > kLoopOverrunUs) {
            uint8_t fd[8];
            packFault(fd, kFaultLoopOverrun, 1, (uint8_t)(elapsed >> 4));
            send(kIdFault, fd);
        }

        // Pace to the fixed rate from t0, not from now, so drift cannot
        // accumulate across a session.
        uint32_t next = t0 + kControlPeriodUs;
        while ((int32_t)(micros() - next) < 0) {
            // spin — a delay() here would allow a tick to be skipped
        }
        (void)lastCycleUs;
        ++seq;
        esp_task_wdt_reset();
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(kLedGpio, OUTPUT);

    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);
    ledcAttachPin(kPwmGpioFwd, 0);
    ledcAttachPin(kPwmGpioRev, 1);

    analogReadResolution(12);

    initCapture();
    initCan();

    g_slip.begin();
    g_abs.begin();
    g_pressure.begin();

    esp_task_wdt_init(1, true);
    esp_task_wdt_add(nullptr);

    // Pin the control task to core 0 so nothing on core 1 can delay it.
    xTaskCreatePinnedToCore(controlTask, "abs", 8192, nullptr,
                            configMAX_PRIORITIES - 2, nullptr, 0);
}

void loop() {
    // Deliberately empty. All real work is in the control task.
    vTaskDelay(portMAX_DELAY);
}
