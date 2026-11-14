// C ABI bridge exposing the firmware HapticEngine and the shared Plant to Python (ctypes).
// Built automatically by telemetry/scripts/native_build.py into firmware/sim/build/
// as libhaptic.dll (Windows), libhaptic.dylib (macOS) or libhaptic.so (Linux).

#include "../src/admittance.h"
#include "plant.h"

#if defined(_WIN32)
#define HB_API extern "C" __declspec(dllexport)
#else
#define HB_API extern "C" __attribute__((visibility("default")))
#endif

HB_API int hb_abi_version() { return 2; }

// ---------------- Engine ----------------
HB_API void* hb_engine_create() { return new HapticEngine(); }

HB_API void* hb_engine_create_custom(float kt_eff, float r_ohm, float v_supply, float i_max, float bemf_comp) {
    ActuatorParams p = { kt_eff, r_ohm, v_supply, i_max, bemf_comp };
    return new HapticEngine(p);
}

HB_API void hb_engine_destroy(void* e) { delete static_cast<HapticEngine*>(e); }

HB_API void hb_engine_set_mode(void* e, char mode, int32_t counts) {
    static_cast<HapticEngine*>(e)->setMode(mode, counts);
}

HB_API void hb_engine_reset(void* e, int32_t counts) { static_cast<HapticEngine*>(e)->reset(counts); }

HB_API void hb_engine_set_pc_enabled(void* e, int32_t on) {
    static_cast<HapticEngine*>(e)->setPassivityControllerEnabled(on != 0);
}

// Returns duty in [-1, 1]; *coast receives 1 when the bridge is high-impedance.
HB_API float hb_engine_step(void* e, uint16_t grip_adc, int32_t counts, int32_t* coast) {
    const MotorCommand c = static_cast<HapticEngine*>(e)->step(grip_adc, counts);
    if (coast) *coast = c.coast ? 1 : 0;
    return c.coast ? 0.0f : c.duty;
}

// out[0..10] = position, velocity, damping, difficulty, tremor_rms, grip_N, energy_J, beta,
//              force_cmd, pc_activations, fault
HB_API void hb_engine_state(void* e, double* out) {
    const HapticEngine* h = static_cast<HapticEngine*>(e);
    out[0] = h->position();  out[1] = h->velocity();  out[2] = h->damping();
    out[3] = h->difficulty(); out[4] = h->tremorRms(); out[5] = h->gripNewtons();
    out[6] = h->observerEnergy(); out[7] = h->passivityBeta(); out[8] = h->forceCommand();
    out[9] = (double)h->pcActivations(); out[10] = (double)h->fault();
}

// ---------------- Plant ----------------
HB_API void* hb_plant_create() { return new Plant(); }
HB_API void hb_plant_destroy(void* p) { delete static_cast<Plant*>(p); }
HB_API void hb_plant_reset(void* p) { static_cast<Plant*>(p)->reset(); }

HB_API void hb_plant_advance(void* p, float fx, float x_eq, float kx, float bx,
                             float fy, float y_eq, float ky, float by, float duty, int32_t coast) {
    HandInput h;
    h.fx = fx; h.x_eq = x_eq; h.kx = kx; h.bx = bx;
    h.fy = fy; h.y_eq = y_eq; h.ky = ky; h.by = by;
    MotorCommand c; c.duty = duty; c.coast = coast != 0;
    static_cast<Plant*>(p)->advance(h, c);
}

// out[0..10] = x, vx, y, vy, lat_err, motor_force, motor_current, contacts, contact_now, finished, motor_work_j
HB_API void hb_plant_state(void* p, double* out) {
    const Plant* pl = static_cast<Plant*>(p);
    const PlantState& s = pl->state();
    out[0] = s.x; out[1] = s.vx; out[2] = s.y; out[3] = s.vy; out[4] = pl->lateralError();
    out[5] = s.motor_force; out[6] = s.motor_current; out[7] = s.contacts;
    out[8] = s.contact_now ? 1.0 : 0.0; out[9] = pl->finished() ? 1.0 : 0.0; out[10] = s.motor_work_j;
}

// Prescribed-motion helper for verification: moves the carriage kinematically (infinitely stiff hand)
// to x_target over one control period while integrating the motor's work exactly.
HB_API void hb_plant_prescribe(void* p, double x_target, float duty, int32_t coast) {
    Plant* pl = static_cast<Plant*>(p);
    MotorCommand c; c.duty = duty; c.coast = coast != 0;
    pl->prescribe(x_target, c);
}

HB_API int32_t hb_plant_encoder(void* p) { return static_cast<Plant*>(p)->encoderCounts(); }
HB_API float hb_plant_wire_y(void* p, float x) { return static_cast<Plant*>(p)->wireY(x); }

// ---------------- Configuration constants (single source of truth: config.h) ----------------
// out[0..13] = counts_per_meter, control_dt, damping_min, damping_max, damping_fixed, tremor_floor,
//              tremor_span, motor_kt_eff, motor_r, supply_v, i_max, vel_deadband, pc_beta_max, b_friction_est
HB_API void hb_config(double* out) {
    out[0] = COUNTS_PER_METER; out[1] = CONTROL_DT_SEC; out[2] = DAMPING_MIN; out[3] = DAMPING_MAX;
    out[4] = DAMPING_FIXED; out[5] = TREMOR_RMS_FLOOR; out[6] = TREMOR_RMS_SPAN; out[7] = MOTOR_KT_EFF;
    out[8] = MOTOR_R_OHM; out[9] = MOTOR_SUPPLY_V; out[10] = DRIVER_I_MAX; out[11] = VEL_DEADBAND;
    out[12] = PC_BETA_MAX; out[13] = B_FRICTION_EST;
}
