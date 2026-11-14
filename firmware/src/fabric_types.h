#pragma once
// Types shared by src.ino functions. Kept in a header because the .ino preprocessor emits function
// prototypes above any type defined in the sketch itself.
#include <Arduino.h>
#include "config.h"

// One control-tick snapshot, written by the control ISR under a sequence lock.
struct Snapshot {
    uint32_t tick;
    uint16_t grip_adc;
    int32_t counts;
    char mode;
    float vel, damping, difficulty, tremor, duty, energy;
    int32_t contacts;
    uint32_t pc_activations, fault;
};

// Fixed-point CSV formatting (no dependence on printf float support).
struct RowWriter {
    char buf[TELEMETRY_ROW_MAX];
    uint16_t n = 0;
    void ch(char c) { if (n < sizeof(buf) - 1) buf[n++] = c; }
    void str(const char* s) { while (*s) ch(*s++); }
    void u32(uint32_t v) {
        char t[11]; int k = 0;
        do { t[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 10);
        while (k) ch(t[--k]);
    }
    void i32(int32_t v) { if (v < 0) { ch('-'); u32((uint32_t)(-(int64_t)v)); } else { u32((uint32_t)v); } }
    void fixed(float v, uint8_t decimals) {
        static const int32_t pow10[] = {1, 10, 100, 1000, 10000, 100000};
        const int32_t scale = pow10[decimals];
        int32_t s = (int32_t)lroundf(v * (float)scale);
        if (s < 0) { ch('-'); s = -s; }
        u32((uint32_t)(s / scale));
        if (decimals) {
            ch('.');
            int32_t frac = s % scale;
            for (int32_t d = scale / 10; d > 0; d /= 10) { ch((char)('0' + (frac / d) % 10)); }
        }
    }
    void sep() { ch(','); }
};
