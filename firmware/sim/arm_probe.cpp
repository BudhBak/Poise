// Cortex-M4 cycle-budget probe (compiled by telemetry/scripts/arm_cycle_budget.py with the Arduino
// renesas_uno flags; never linked into the firmware). Each probe_* wraps one 500 Hz step().
#include "innovation_engine.h"

static InnovationConfig worstCase() {          // every feature enabled at once
    InnovationConfig c;
    c.tremor_channel = 1; c.tremor_force = true; c.fractional = true;
    c.b_vol_lo = 2.5f; c.b_vol_hi = 5.0f; c.b_tr_lo = 22.0f; c.b_tr_hi = 45.0f;
    c.m_virt_max = 1.0f; c.f0_tracking = c.anticipator = c.rho_gate = true;
    c.dither_duty = 0.025f; c.swell_amp = 0.2f;
    return c;
}
static InnovationConfig dualBand() {
    InnovationConfig c; c.tremor_channel = 1; c.b_vol_lo = c.b_vol_hi = 3.75f; c.b_tr_lo = c.b_tr_hi = 33.5f;
    return c;
}

HapticEngine g_v2;
InnovationEngine g_worst(worstCase());
InnovationEngine g_db(dualBand());

extern "C" __attribute__((noinline)) float probe_v2(uint16_t a, int32_t c)    { return g_v2.step(a, c).duty; }
extern "C" __attribute__((noinline)) float probe_worst(uint16_t a, int32_t c) { return g_worst.step(a, c).duty; }
extern "C" __attribute__((noinline)) float probe_db(uint16_t a, int32_t c)    { return g_db.step(a, c).duty; }
