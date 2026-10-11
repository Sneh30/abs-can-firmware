// can_pack.h — byte-exact encode/decode.
//
// Explicit shifts, no struct punning, no endianness assumptions. The wire
// layout is a contract with the dashboard and the logger; it must not be
// able to change because someone compiled with a different alignment.

#pragma once

#include <stdint.h>
#include <string.h>

#include "can_ids.h"

namespace absfw {

inline void putU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v        & 0xFF);
    p[1] = (uint8_t)((v >> 8)  & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

inline void putU16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v        & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

inline uint32_t getU32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

inline uint16_t getU16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

struct WheelFrame {
    uint32_t period_us;
    uint16_t ticks;
    uint8_t  flags;
    uint8_t  seq;
};

inline void packWheel(uint8_t d[8], const WheelFrame& f) {
    putU32(&d[0], f.period_us);
    putU16(&d[4], f.ticks);
    d[6] = f.flags;
    d[7] = f.seq;
}

inline WheelFrame unpackWheel(const uint8_t d[8]) {
    WheelFrame f{};
    f.period_us = getU32(&d[0]);
    f.ticks     = getU16(&d[4]);
    f.flags     = d[6];
    f.seq       = d[7];
    return f;
}

struct VehicleRefFrame {
    uint32_t  v_ref_mm_s;
    uint8_t   source;
    uint8_t   quality;
};

inline void packVehicleRef(uint8_t d[8], const VehicleRefFrame& f) {
    putU32(&d[0], f.v_ref_mm_s);
    d[4] = f.source;
    d[5] = f.quality;
    d[6] = 0;
    d[7] = 0;
}

inline VehicleRefFrame unpackVehicleRef(const uint8_t d[8]) {
    VehicleRefFrame f{};
    f.v_ref_mm_s = getU32(&d[0]);
    f.source     = d[4];
    f.quality    = d[5];
    return f;
}

struct AbsStateFrame {
    uint8_t  state;
    uint8_t  active_axle;
    uint8_t  lambda_fp;
    uint16_t cycles;
    uint16_t flags;
};

inline void packAbsState(uint8_t d[8], const AbsStateFrame& f) {
    d[0] = f.state;
    d[1] = f.active_axle;
    d[2] = f.lambda_fp;
    putU16(&d[3], f.cycles);
    putU16(&d[5], f.flags);
    d[7] = 0;
}

inline AbsStateFrame unpackAbsState(const uint8_t d[8]) {
    AbsStateFrame f{};
    f.state        = d[0];
    f.active_axle  = d[1];
    f.lambda_fp    = d[2];
    f.cycles       = getU16(&d[3]);
    f.flags        = getU16(&d[5]);
    return f;
}

inline void packFault(uint8_t d[8], uint16_t code, uint8_t severity,
                      uint8_t context) {
    putU16(&d[0], code);
    d[2] = severity;
    d[3] = context;
    putU32(&d[4], 0);
}

}  // namespace absfw
