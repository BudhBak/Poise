#pragma once
// SymbioGrip HapticEngine: 500 Hz adaptive viscous damping with time-domain passivity.
//
// One source file is shared verbatim by the RA4M1 firmware (src.ino), the multi-rate SIL
// simulation (firmware/sim/test_bench.cpp) and the desktop bench bridge (firmware/sim/hil_bridge.cpp).
//
// Blindness contract: the engine's only inputs are (grip ADC counts, quadrature encoder counts) at a
// fixed 500 Hz rate, plus the experimenter-selected mode. It has no access to task geometry, wall
// positions, lateral error, trial identity or time.
//
// Plain C++11, no Arduino dependencies.

#include <stdint.h>
#include <math.h>
#include "config.h"

// Actuator model (see config.h): voltage-driven DC motor seen at the carriage.
//   force = kt_eff * I,   I = (V - ke * v) / R,   ke == kt_eff (SI),   b_emf = kt_eff^2 / R
struct ActuatorParams {
    float kt_eff;     // N/A
    float r_ohm;      // Ohm
    float v_supply;   // V
    float i_max;      // A
    float bemf_comp;  // 0..1, fraction of back-EMF cancelled in software
    float bEmf() const { return kt_eff * kt_eff / r_ohm; }
};

inline ActuatorParams defaultActuatorParams() {
    ActuatorParams p = { MOTOR_KT_EFF, MOTOR_R_OHM, MOTOR_SUPPLY_V, DRIVER_I_MAX, BEMF_COMPENSATION };
    return p;
}

// duty in [-1, 1]: sign = direction of commanded force along +x. coast = bridge high-impedance.
struct MotorCommand {
    float duty;
    bool  coast;
};

enum EngineFault : uint8_t {
    FAULT_NONE         = 0,
    FAULT_OVERSPEED    = 1,
    FAULT_ENCODER_JUMP = 2
};

class HapticEngine {
public:
    // ---- Filter coefficients (designed in telemetry/scripts/verify_control_theory.py, which
    //      parses these lines and fails if they drift from the specification) ----
    // 2nd-order Butterworth low-pass on velocity, fc = 25 Hz, fs = 500 Hz, prewarped bilinear.
    static constexpr float LP_B0 = 0.020083366f, LP_B1 = 0.040166731f, LP_B2 = 0.020083366f;
    static constexpr float LP_A1 = -1.561018076f, LP_A2 = 0.641351538f;
    // 1st-order Butterworth high-pass on acceleration, fc = 5 Hz, fs = 500 Hz.
    static constexpr float HP_B0 = 0.969531253f, HP_B1 = -0.969531253f, HP_A1 = -0.939062506f;
    // Critically damped damping smoother, wn = 36 rad/s, zeta = 1, prewarped bilinear, unity DC.
    static constexpr float SM_B0 = 1.208503157e-03f, SM_B1 = 2.417006313e-03f, SM_B2 = 1.208503157e-03f;
    static constexpr float SM_A1 = -1.860945872f, SM_A2 = 0.865779885f;

    explicit HapticEngine(const ActuatorParams& p = defaultActuatorParams())
        : act_(p), mode_('C'), pc_activations_(0), pc_enabled_(true), fixed_b_(DAMPING_FIXED) {
        reset(0);
    }

    // Verification only (verify_control_theory.py compares with/without). The firmware never disables it.
    void setPassivityControllerEnabled(bool on) { pc_enabled_ = on; }

    // Simulation only (test_bench --b-fixed): dose-matched control condition. Firmware keeps DAMPING_FIXED.
    void setFixedDamping(float b) {
        fixed_b_ = b < DAMPING_MIN ? DAMPING_MIN : (b > DAMPING_MAX ? DAMPING_MAX : b);
    }

    // Select experimental condition. Always performs a full state reset at the given encoder count.
    void setMode(char mode, int32_t encoder_counts_now) {
        if (mode == 'A' || mode == 'B' || mode == 'C') {
            mode_ = mode;
        }
        reset(encoder_counts_now);
    }

    // Full state hygiene: every filter, estimator, observer and latch returns to its initial value.
    void reset(int32_t encoder_counts_now) {
        prev_counts_ = encoder_counts_now;
        pos_m_ = (float)encoder_counts_now / COUNTS_PER_METER;
        first_sample_ = true;

        vel_raw_ = vel_f_ = prev_vel_f_ = 0.0f;
        lp_x1_ = lp_x2_ = lp_y1_ = lp_y2_ = 0.0f;
        prev_acc_ = acc_hp_ = 0.0f;
        tremor_ms_ = tremor_rms_ = 0.0f;

        grip_n_ = 0.0f;
        grip_norm_ = tremor_norm_ = difficulty_ = 0.0f;

        const float b0 = initialDamping();
        slewed_b_ = damping_ = b0;
        sm_x1_ = sm_x2_ = sm_y1_ = sm_y2_ = b0;

        beta_ = 0.0f;
        energy_j_ = 0.0f;
        prev_volts_ = 0.0f;
        prev_coast_ = true;
        force_cmd_ = 0.0f;
        last_cmd_.duty = 0.0f;
        last_cmd_.coast = true;
        fault_ = FAULT_NONE;
        pc_activations_ = 0;
    }

    // One 500 Hz control step. Inputs are raw sensor counts only.
    MotorCommand step(uint16_t grip_adc, int32_t encoder_counts) {
        const float dt = CONTROL_DT_SEC;

        if (first_sample_) {               // no derivative across a reset boundary
            prev_counts_ = encoder_counts;
        }
        const int32_t dc = encoder_counts - prev_counts_;
        prev_counts_ = encoder_counts;
        const float dx = (float)dc / COUNTS_PER_METER;
        pos_m_ = (float)encoder_counts / COUNTS_PER_METER;

        if (dc > ENCODER_MAX_JUMP || dc < -ENCODER_MAX_JUMP) {
            fault_ = FAULT_ENCODER_JUMP;
        }

        // ---- 1. Passivity observer, exact for the zero-order-held interval that just ended ----
        // Work done BY the motor on the carriage over [t_k, t_k+1] with constant terminal voltage V:
        //   E_out = (kt/R) V dx - b_emf * integral(v^2 dt)  <=  (kt/R) V dx - b_emf dx^2 / T
        // (Cauchy-Schwarz: integral(v^2) >= dx^2 / T). W accumulates energy absorbed by the device.
        if (!prev_coast_) {
            const float e_out_upper = (act_.kt_eff / act_.r_ohm) * prev_volts_ * dx
                                    - act_.bEmf() * dx * dx / dt;
            energy_j_ -= e_out_upper;
            if (energy_j_ > PO_ENERGY_CAP_J) energy_j_ = PO_ENERGY_CAP_J;
        }

        // ---- 2. Velocity: backward difference + 25 Hz Butterworth ----
        vel_raw_ = dx / dt;
        vel_f_ = LP_B0 * vel_raw_ + LP_B1 * lp_x1_ + LP_B2 * lp_x2_ - LP_A1 * lp_y1_ - LP_A2 * lp_y2_;
        lp_x2_ = lp_x1_; lp_x1_ = vel_raw_;
        lp_y2_ = lp_y1_; lp_y1_ = vel_f_;

        // ---- 3. Tremor: 5-25 Hz acceleration band, mean-square EMA (tau = 250 ms) ----
        const float acc = first_sample_ ? 0.0f : (vel_f_ - prev_vel_f_) / dt;
        acc_hp_ = HP_B0 * acc + HP_B1 * prev_acc_ - HP_A1 * acc_hp_;
        prev_acc_ = acc;
        prev_vel_f_ = vel_f_;
        tremor_ms_ += TREMOR_MS_ALPHA * (acc_hp_ * acc_hp_ - tremor_ms_);
        tremor_rms_ = sqrtf(tremor_ms_);

        // ---- 4. Grip: FSR divider inverse model, 20 ms EMA ----
        float a = (float)grip_adc / ADC_FULL_SCALE;
        if (a > 0.995f) a = 0.995f;
        if (a < 0.0f) a = 0.0f;
        const float grip_inst_n = FSR_FORCE_SCALE_N * a / (1.0f - a);
        grip_n_ = first_sample_ ? grip_inst_n : grip_n_ + GRIP_LP_ALPHA * (grip_inst_n - grip_n_);

        grip_norm_ = clamp01((grip_n_ - GRIP_REST_N) / (GRIP_FULL_N - GRIP_REST_N));
        tremor_norm_ = clamp01((tremor_rms_ - TREMOR_RMS_FLOOR) / TREMOR_RMS_SPAN);
        difficulty_ = clamp01(DIFF_W_GRIP * grip_norm_ + DIFF_W_TREMOR * tremor_norm_);
        first_sample_ = false;

        // ---- 5. Damping law ----
        if (mode_ == 'C') {
            const float b_target = DAMPING_MIN + difficulty_ * (DAMPING_MAX - DAMPING_MIN);
            const float max_rise = DAMPING_SLEW_ATTACK * dt;
            const float max_fall = DAMPING_SLEW_RELEASE * dt;
            if (b_target - slewed_b_ > max_rise)       slewed_b_ += max_rise;
            else if (slewed_b_ - b_target > max_fall)  slewed_b_ -= max_fall;
            else                                       slewed_b_ = b_target;

            damping_ = SM_B0 * slewed_b_ + SM_B1 * sm_x1_ + SM_B2 * sm_x2_ - SM_A1 * sm_y1_ - SM_A2 * sm_y2_;
            sm_x2_ = sm_x1_; sm_x1_ = slewed_b_;
            sm_y2_ = sm_y1_; sm_y1_ = damping_;
        } else {
            damping_ = initialDamping();
        }

        // ---- 6. Passivity controller: dissipate any observed energy deficit ----
        // Acts on the UNFILTERED velocity (backward difference): a damping term applied through the 25 Hz
        // low-pass would itself be active wherever the filter's phase lag exceeds 90 degrees.
        beta_ = 0.0f;
        if (pc_enabled_ && mode_ != 'A' && energy_j_ < 0.0f && fabsf(vel_raw_) > VEL_DEADBAND) {
            beta_ = -energy_j_ / (vel_raw_ * vel_raw_ * dt);
            if (beta_ > PC_BETA_MAX) beta_ = PC_BETA_MAX;
            pc_activations_++;
        }

        // ---- 7. Actuator: force -> current -> voltage with back-EMF compensation -> duty ----
        if (fabsf(vel_f_) > VEL_FAULT) fault_ = FAULT_OVERSPEED;
        force_cmd_ = -damping_ * vel_f_ - beta_ * vel_raw_;

        MotorCommand cmd;
        cmd.duty = 0.0f;
        cmd.coast = true;
        const bool drive = (mode_ != 'A') && (fault_ == FAULT_NONE) &&
                           (fabsf(vel_f_) >= VEL_DEADBAND || beta_ > 0.0f);
        if (drive) {
            float i_cmd = force_cmd_ / act_.kt_eff;
            if (i_cmd >  act_.i_max) i_cmd =  act_.i_max;
            if (i_cmd < -act_.i_max) i_cmd = -act_.i_max;
            // While the observer reports an energy deficit (beta > 0) the back-EMF feed-forward -- the only
            // term that can make the device active (e.g. if K_t is over-estimated) -- is dropped. Plain
            // voltage drive keeps the motor's own electrical damping K_t^2/R in the loop and is passive for
            // every B + beta <= DAMPING_MAX + PC_BETA_MAX (verify_control_theory.py, check V5 fallback).
            const float comp = (beta_ > 0.0f) ? 0.0f : act_.bemf_comp;
            const float volts = i_cmd * act_.r_ohm + comp * act_.kt_eff * vel_f_;
            float duty = volts / act_.v_supply;
            if (duty >  1.0f) duty =  1.0f;
            if (duty < -1.0f) duty = -1.0f;
            cmd.duty = duty;
            cmd.coast = false;
        } else {
            force_cmd_ = 0.0f;
        }
        prev_volts_ = cmd.coast ? 0.0f : cmd.duty * act_.v_supply;
        prev_coast_ = cmd.coast;
        last_cmd_ = cmd;
        return cmd;
    }

    // ---- Accessors ----
    char  mode() const            { return mode_; }
    float position() const        { return pos_m_; }
    float velocity() const        { return vel_f_; }
    float damping() const         { return (mode_ == 'A') ? 0.0f : damping_ + beta_; }
    float passivityBeta() const   { return beta_; }
    float difficulty() const      { return difficulty_; }
    float tremorRms() const       { return tremor_rms_; }
    float gripNewtons() const     { return grip_n_; }
    float observerEnergy() const  { return energy_j_; }
    float forceCommand() const    { return force_cmd_; }
    MotorCommand lastCommand() const { return last_cmd_; }
    uint8_t fault() const         { return (uint8_t)fault_; }
    uint32_t pcActivations() const { return pc_activations_; }
    const ActuatorParams& actuator() const { return act_; }

private:
    static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
    float initialDamping() const {
        return (mode_ == 'A') ? 0.0f : ((mode_ == 'B') ? fixed_b_ : DAMPING_MIN);
    }

    ActuatorParams act_;
    char mode_;

    int32_t prev_counts_;
    bool first_sample_;
    float pos_m_;
    float vel_raw_, vel_f_, prev_vel_f_;
    float lp_x1_, lp_x2_, lp_y1_, lp_y2_;
    float prev_acc_, acc_hp_;
    float tremor_ms_, tremor_rms_;
    float grip_n_, grip_norm_, tremor_norm_, difficulty_;
    float slewed_b_, damping_;
    float sm_x1_, sm_x2_, sm_y1_, sm_y2_;
    float beta_, energy_j_;
    float prev_volts_;
    bool prev_coast_;
    float force_cmd_;
    MotorCommand last_cmd_;
    EngineFault fault_;
    uint32_t pc_activations_;
    bool pc_enabled_;
    float fixed_b_;
};
