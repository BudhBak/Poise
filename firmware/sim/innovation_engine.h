#pragma once
// SymbioGrip R&D candidate engine (v3 innovation lab). NOT part of the firmware unless a feature is
// promoted by the pre-registered rule in docs/InnovationResearch.md.
//
// Same contract as HapticEngine (firmware/src/admittance.h): inputs are (grip ADC counts, encoder counts)
// at 500 Hz only; no geometry, wall, trial or time information. Float-only C++, no heap, bounded loops
// (16 iterations), so it compiles for the RA4M1 and can be cycle-counted (telemetry/scripts/arm_cycle_budget.py).
//
// Shared verbatim with v2 (copied semantics, checked by the parity test in innovation_lab.cpp):
//   exact ZOH passivity observer, 25 Hz velocity LPF, FSR inverse model + 20 ms EMA, raw-velocity passivity
//   controller with back-EMF feed-forward fallback, current limit, deadband, overspeed latch.
//
// Candidate features (each independently switchable through InnovationConfig):
//   1  BMFLC: 16 LMS-adapted quadrature bins 6.0..13.5 Hz on 4 Hz-high-passed raw velocity; per-bin complex
//      correction undoes the high-pass and backward-difference responses and advances T/2, giving a
//      phase-compensated tremor velocity v_tr, tremor acceleration a_tr and the analytic-signal envelope.
//   1b LTI alternative: one RBJ band-pass biquad (9 Hz, Q 1.2) on raw velocity (ablation of 1).
//   2  dual-band impedance  F = -B_vol v_f - (B_tr - B_vol) v_tr
//   3  short-memory Grunwald-Letnikov fractional damper  F = -B_vol * N(alpha) * D^(alpha-1) v_raw, 16 taps
//   4  tank-gated virtual inertia  F += -M_virt(W) a_tr
//   5  grip-surge anticipator (band-limited dF/dt) and grip-tremor correlation gate rho_GT
//   6  asymmetric baseline tracker F0 (tau_down 1.5 s, tau_up 25 s), tank-gated dither, 0.1 Hz damping swell

#include <stdint.h>
#include <math.h>
#include "../src/config.h"
#include "../src/admittance.h"

struct InnovationConfig {
    float b_vol_lo = 3.75f, b_vol_hi = 3.75f;   // voluntary damping range (lo = hi -> fixed)
    uint8_t tremor_channel = 0;                  // 0 none, 1 BMFLC, 2 LTI band-pass
    bool  tremor_force = true;                   // false: tremor channel used for sensing only
    bool  actuate = true;                        // false: bridge always coasts (sensing-only, like Mode A)
    float b_tr_lo = 0.0f, b_tr_hi = 0.0f;        // tremor-band damping range
    bool  fractional = false;                    // replace B_vol*v_f by the fractional damper
    float fo_alpha_lo = 0.55f, fo_alpha_hi = 0.95f;
    float m_virt_max = 0.0f;                     // kg (BMFLC only)
    bool  f0_tracking = false;
    bool  anticipator = false;
    bool  rho_gate = false;
    float dither_duty = 0.0f;                    // amplitude of uniform duty dither (fraction of full scale)
    float swell_amp = 0.0f;                      // relative B_vol modulation of the 0.1 Hz cue
};

class InnovationEngine {
public:
    static const int BM_N = 16;
    static const int FO_N = 16;
    static constexpr float BM_F0_HZ = 6.0f, BM_DF_HZ = 0.5f;
#ifndef BM_MU_OVERRIDE
    static constexpr float BM_MU = 0.00314f;   // per-bin bandwidth ~ bin spacing: mu = pi * df * T
#else
    static constexpr float BM_MU = BM_MU_OVERRIDE;
#endif
    static constexpr float HP_FC_HZ = 4.0f;
#ifndef BM_JITTER_OVERRIDE
    static constexpr float BM_JITTER = 0.0f;   // incommensurate-grid option (tested, not used: see InnovationResearch.md)
#else
    static constexpr float BM_JITTER = BM_JITTER_OVERRIDE;
#endif
#ifndef BM_LEAK_OVERRIDE
    static constexpr float BM_LEAK = 0.001f;   // leaky LMS (tau 2 s): bounds lightly damped closed-loop modes (|p| = 0.99944, tau 3.6 s)
#else
    static constexpr float BM_LEAK = BM_LEAK_OVERRIDE;
#endif
    static constexpr float FO_FREF_HZ = 2.0f;
    // Tremor-envelope normalisation (m/s RMS of v_tr). Calibrated BEFORE any confirmatory run from
    // (a) quantisation-only glides and (b) Mode A on development seed 1001; see docs/InnovationResearch.md section 5.
    static constexpr float TR_ENV_FLOOR = 0.0086f;   // >= quantisation-only maximum 0.00852 m/s (CAL, seed 1001)
    static constexpr float TR_ENV_SPAN  = 0.0086f;   // full scale at 2x floor (Mode A dev median 0.0022, p90 0.0042)
    static constexpr float SURGE_REF_NPS = 10.0f;   // N/s grip rise mapped to full anticipation
    static constexpr float VI_MARGIN_J = 0.02f;     // tank energy kept in reserve before inertia is rendered
    static constexpr float CUE_PERIOD_S = 10.0f, CUE_INHALE_S = 4.0f;

    explicit InnovationEngine(const InnovationConfig& c = InnovationConfig(),
                              const ActuatorParams& a = defaultActuatorParams())
        : cfg_(c), act_(a), pc_enabled_(true), frozen_(false), frozen_d_(0.0f) {
        designFilters();
        reset(0);
    }

    void setPassivityControllerEnabled(bool on) { pc_enabled_ = on; }
    // Verification hook: freeze the difficulty d (adaptive laws become LTI) for frequency-response tests.
    void freezeDifficulty(bool on, float d) { frozen_ = on; frozen_d_ = d; }

    void reset(int32_t counts) {
        prev_counts_ = counts; first_ = true;
        vel_raw_ = vel_f_ = 0.0f;
        lp_x1_ = lp_x2_ = lp_y1_ = lp_y2_ = 0.0f;
        hp_x1_ = hp_x2_ = hp_y1_ = hp_y2_ = 0.0f;
        bp_x1_ = bp_x2_ = bp_y1_ = bp_y2_ = 0.0f;
        bp_ms_ = 0.0f;
        for (int k = 0; k < BM_N; ++k) {
            ws_[k] = wc_[k] = 0.0f;
            osc_s_[k] = 0.0f; osc_c_[k] = 1.0f;
        }
        for (int j = 0; j < FO_N; ++j) fo_hist_[j] = 0.0f;
        v_tr_ = a_tr_ = env_ = 0.0f;
        grip_n_ = grip_prev_ = dgrip_f_ = 0.0f; f0_ = GRIP_REST_N;
        m_g_ = m_t_ = c_gt_ = v_g_ = v_t_ = rho_ = 0.0f;
        d_ = tremor_norm_ = grip_rel_ = boost_ = 0.0f;
        slewed_b_ = cfg_.b_vol_lo;
        sm_x1_ = sm_x2_ = sm_y1_ = sm_y2_ = cfg_.b_vol_lo;
        b_vol_ = cfg_.b_vol_lo; b_tr_ = cfg_.b_tr_lo; d_tr_f_ = 0.0f;
        alpha_ = cfg_.fo_alpha_hi; fo_out_ = 0.0f; m_virt_ = 0.0f;
        beta_ = energy_j_ = prev_volts_ = 0.0f; prev_coast_ = true;
        force_cmd_ = force_lin_ = 0.0f;
        tick_ = 0; rng_ = 0x2545F491u;
        fault_ = FAULT_NONE; pc_activations_ = 0;
    }

    MotorCommand step(uint16_t grip_adc, int32_t counts) {
        if (first_) prev_counts_ = counts;
        const int32_t dc = counts - prev_counts_;
        prev_counts_ = counts;
        if (dc > ENCODER_MAX_JUMP || dc < -ENCODER_MAX_JUMP) fault_ = FAULT_ENCODER_JUMP;
        return stepCore((float)dc / COUNTS_PER_METER, grip_adc);
    }

    // Core step on an exact position increment (the frequency-response harness calls this directly).
    MotorCommand stepCore(float dx, uint16_t grip_adc) {
        const float T = CONTROL_DT_SEC;

        // ---- 1. passivity observer (identical to v2) ----
        if (!prev_coast_) {
            const float e_out_upper = (act_.kt_eff / act_.r_ohm) * prev_volts_ * dx - act_.bEmf() * dx * dx / T;
            energy_j_ -= e_out_upper;
            if (energy_j_ > PO_ENERGY_CAP_J) energy_j_ = PO_ENERGY_CAP_J;
        }

        // ---- 2. velocity (identical to v2) ----
        vel_raw_ = dx / T;
        vel_f_ = HapticEngine::LP_B0 * vel_raw_ + HapticEngine::LP_B1 * lp_x1_ + HapticEngine::LP_B2 * lp_x2_
               - HapticEngine::LP_A1 * lp_y1_ - HapticEngine::LP_A2 * lp_y2_;
        lp_x2_ = lp_x1_; lp_x1_ = vel_raw_; lp_y2_ = lp_y1_; lp_y1_ = vel_f_;

        // ---- 3. tremor channel ----
        if (cfg_.tremor_channel == 1) bmflc(vel_raw_);
        else if (cfg_.tremor_channel == 2) bandpass(vel_raw_);
        else { v_tr_ = a_tr_ = 0.0f; env_ = 0.0f; }

        // ---- 4. grip: FSR inverse model + 20 ms EMA (identical to v2), baseline, surge, correlation ----
        float a = (float)grip_adc / ADC_FULL_SCALE;
        if (a > 0.98f) a = 0.98f;
        const float g_inst = FSR_FORCE_SCALE_N * a / (1.0f - a);
        grip_n_ = first_ ? g_inst : grip_n_ + GRIP_LP_ALPHA * (g_inst - grip_n_);
        if (first_) { grip_prev_ = grip_n_; f0_ = cfg_.f0_tracking ? grip_n_ : GRIP_REST_N; }
        if (cfg_.f0_tracking) {
            const float tau = (grip_n_ < f0_) ? 1.5f : 25.0f;
            f0_ += (grip_n_ - f0_) * (T / tau);
        }
        grip_rel_ = clamp01((grip_n_ - f0_) / (GRIP_FULL_N - GRIP_REST_N));
        const float dg = (grip_n_ - grip_prev_) / T;
        grip_prev_ = grip_n_;
        dgrip_f_ += 0.0609f * (dg - dgrip_f_);                 // 1st-order 5 Hz: 1 - exp(-2*pi*5*T)
        tremor_norm_ = clamp01((env_ - TR_ENV_FLOOR) / TR_ENV_SPAN);
        // grip-tremor correlation, EMA tau = 2 s
        const float ka = 0.001f;
        m_g_ += ka * (grip_rel_ - m_g_);
        m_t_ += ka * (tremor_norm_ - m_t_);
        const float dgv = grip_rel_ - m_g_, dtv = tremor_norm_ - m_t_;
        c_gt_ += ka * (dgv * dtv - c_gt_);
        v_g_ += ka * (dgv * dgv - v_g_);
        v_t_ += ka * (dtv * dtv - v_t_);
        rho_ = c_gt_ / sqrtf(v_g_ * v_t_ + 1e-6f);
        const float rho_pos = rho_ > 0.0f ? (rho_ > 1.0f ? 1.0f : rho_) : 0.0f;
        const float surge = cfg_.anticipator ? clamp01(dgrip_f_ / SURGE_REF_NPS) : 0.0f;
        boost_ = cfg_.rho_gate ? surge * rho_pos : surge;

        // ---- 5. difficulty ----
        if (cfg_.rho_gate) d_ = clamp01(tremor_norm_ * (1.0f + rho_pos * grip_rel_) + 0.5f * boost_);
        else               d_ = clamp01(DIFF_W_GRIP * grip_rel_ + DIFF_W_TREMOR * tremor_norm_ + 0.5f * boost_);
        if (frozen_) d_ = frozen_d_;
        first_ = false;

        // ---- 6. voluntary damping (v2 slew + 36 rad/s smoother), tremor damping (20 ms) ----
        const float b_target = cfg_.b_vol_lo + d_ * (cfg_.b_vol_hi - cfg_.b_vol_lo);
        if (frozen_ || cfg_.b_vol_lo == cfg_.b_vol_hi) {
            slewed_b_ = b_vol_ = b_target;
            sm_x1_ = sm_x2_ = sm_y1_ = sm_y2_ = b_target;
        } else {
            const float up = DAMPING_SLEW_ATTACK * T, down = DAMPING_SLEW_RELEASE * T;
            if (b_target - slewed_b_ > up) slewed_b_ += up;
            else if (slewed_b_ - b_target > down) slewed_b_ -= down;
            else slewed_b_ = b_target;
            b_vol_ = HapticEngine::SM_B0 * slewed_b_ + HapticEngine::SM_B1 * sm_x1_ + HapticEngine::SM_B2 * sm_x2_
                   - HapticEngine::SM_A1 * sm_y1_ - HapticEngine::SM_A2 * sm_y2_;
            sm_x2_ = sm_x1_; sm_x1_ = slewed_b_; sm_y2_ = sm_y1_; sm_y1_ = b_vol_;
        }
        d_tr_f_ = frozen_ ? d_ : d_tr_f_ + 0.0952f * (d_ - d_tr_f_);    // 20 ms
        b_tr_ = cfg_.b_tr_lo + d_tr_f_ * (cfg_.b_tr_hi - cfg_.b_tr_lo);
        float b_vol_eff = b_vol_;
        if (cfg_.swell_amp > 0.0f) b_vol_eff *= 1.0f + cfg_.swell_amp * breathingCue();

        // ---- 7. force law ----
        float f = 0.0f;
        if (cfg_.fractional) {
            alpha_ = cfg_.fo_alpha_lo + d_ * (cfg_.fo_alpha_hi - cfg_.fo_alpha_lo);
            fractional(vel_raw_);
            f = -b_vol_eff * fo_out_;
        } else {
            f = -b_vol_eff * vel_f_;
        }
        if (cfg_.tremor_channel != 0 && cfg_.tremor_force) f -= (b_tr_ - b_vol_eff) * v_tr_;
        m_virt_ = 0.0f;
        if (cfg_.m_virt_max > 0.0f && cfg_.tremor_channel == 1) {
            m_virt_ = cfg_.m_virt_max * clamp01((energy_j_ - VI_MARGIN_J) / VI_MARGIN_J);
            if (frozen_) m_virt_ = cfg_.m_virt_max;
            f -= m_virt_ * a_tr_;
        }
        force_lin_ = f;

        // ---- 8. passivity controller (identical to v2) ----
        beta_ = 0.0f;
        if (!frozen_ && pc_enabled_ && energy_j_ < 0.0f && fabsf(vel_raw_) > VEL_DEADBAND) {
            beta_ = -energy_j_ / (vel_raw_ * vel_raw_ * T);
            if (beta_ > PC_BETA_MAX) beta_ = PC_BETA_MAX;
            pc_activations_++;
        }
        if (fabsf(vel_f_) > VEL_FAULT) fault_ = FAULT_OVERSPEED;
        force_cmd_ = f - beta_ * vel_raw_;

        // ---- 9. actuator (identical to v2) + tank-gated dither ----
        MotorCommand cmd; cmd.duty = 0.0f; cmd.coast = true;
        const bool drive = cfg_.actuate && (fault_ == FAULT_NONE) && (fabsf(vel_f_) >= VEL_DEADBAND || beta_ > 0.0f);
        if (drive) {
            float i_cmd = force_cmd_ / act_.kt_eff;
            if (i_cmd >  act_.i_max) i_cmd =  act_.i_max;
            if (i_cmd < -act_.i_max) i_cmd = -act_.i_max;
            const float comp = (beta_ > 0.0f) ? 0.0f : act_.bemf_comp;
            float duty = (i_cmd * act_.r_ohm + comp * act_.kt_eff * vel_f_) / act_.v_supply;
            if (cfg_.dither_duty > 0.0f && energy_j_ > VI_MARGIN_J && beta_ == 0.0f) duty += cfg_.dither_duty * uniformPm1();
            if (duty >  1.0f) duty =  1.0f;
            if (duty < -1.0f) duty = -1.0f;
            cmd.duty = duty; cmd.coast = false;
        } else {
            force_cmd_ = 0.0f;
        }
        prev_volts_ = cmd.coast ? 0.0f : cmd.duty * act_.v_supply;
        prev_coast_ = cmd.coast;
        ++tick_;
        return cmd;
    }

    // ---- accessors ----
    float velocity() const { return vel_f_; }
    float damping() const { return b_vol_ + beta_; }
    float tremorDamping() const { return cfg_.tremor_channel ? b_tr_ : 0.0f; }
    float difficulty() const { return d_; }
    float tremorEnvelope() const { return env_; }
    float tremorVelocity() const { return v_tr_; }
    float grip() const { return grip_n_; }
    float baseline() const { return f0_; }
    float rho() const { return rho_; }
    float virtualMass() const { return m_virt_; }
    float alpha() const { return alpha_; }
    float observerEnergy() const { return energy_j_; }
    float passivityBeta() const { return beta_; }
    float forceCommand() const { return force_cmd_; }
    float forceLinear() const { return force_lin_; }
    uint32_t pcActivations() const { return pc_activations_; }
    uint8_t fault() const { return (uint8_t)fault_; }

private:
    static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    void designFilters() {
        const float T = CONTROL_DT_SEC, PI = 3.14159265358979f;
        // 2nd-order Butterworth high-pass at 4 Hz (prewarped bilinear)
        const float K = tanf(PI * HP_FC_HZ * T), q = 1.41421356f;
        const float n = 1.0f / (1.0f + q * K + K * K);
        hp_b0_ = n; hp_b1_ = -2.0f * n; hp_b2_ = n;
        hp_a1_ = 2.0f * (K * K - 1.0f) * n; hp_a2_ = (1.0f - q * K + K * K) * n;
        // RBJ band-pass (constant 0 dB peak), f0 = 9 Hz, Q = 1.2
        const float w0 = 2.0f * PI * 9.0f * T, al = sinf(w0) / (2.0f * 1.2f), a0 = 1.0f + al;
        bp_b0_ = al / a0; bp_b2_ = -al / a0; bp_a1_ = -2.0f * cosf(w0) / a0; bp_a2_ = (1.0f - al) / a0;
        // BMFLC bins: oscillator rotation and complex output correction K_k = e^{j w T/2} / (H_bd H_hp)
        for (int k = 0; k < BM_N; ++k) {
            // incommensurate grid: golden-ratio offsets (|d| <= 0.06 Hz) so the bins never realign coherently
            const float jit = 0.12f * (0.6180340f * (float)(k + 1) - floorf(0.6180340f * (float)(k + 1))) - 0.06f;
            const float w = 2.0f * PI * (BM_F0_HZ + BM_DF_HZ * k + BM_JITTER * jit);
            const float th = w * T;
            rot_c_[k] = cosf(th); rot_s_[k] = sinf(th); omega_[k] = w;
            // H_bd = (1 - e^{-j th}) / (j th)
            const float br = sinf(th) / th, bi = (cosf(th) - 1.0f) / th;
            // H_hp(e^{j th}) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
            const float c1 = cosf(th), s1 = -sinf(th), c2 = cosf(2 * th), s2 = -sinf(2 * th);
            const float nr = hp_b0_ + hp_b1_ * c1 + hp_b2_ * c2, ni = hp_b1_ * s1 + hp_b2_ * s2;
            const float dr = 1.0f + hp_a1_ * c1 + hp_a2_ * c2, di = hp_a1_ * s1 + hp_a2_ * s2;
            const float dd = dr * dr + di * di;
            const float hr = (nr * dr + ni * di) / dd, hi = (ni * dr - nr * di) / dd;
            const float tr = br * hr - bi * hi, ti = br * hi + bi * hr;         // H_bd * H_hp
            const float ar = cosf(th * 0.5f), ai = sinf(th * 0.5f);             // e^{j th/2}
            const float tt = tr * tr + ti * ti;
            kr_[k] = (ar * tr + ai * ti) / tt;
            ki_[k] = (ai * tr - ar * ti) / tt;
        }
        // fractional-operator reference phasors at FO_FREF_HZ: e^{-j th_ref j}
        const float thr = 2.0f * PI * FO_FREF_HZ * T;
        for (int j = 0; j < FO_N; ++j) { fo_cr_[j] = cosf(thr * j); fo_sr_[j] = -sinf(thr * j); }
    }

    void bmflc(float v) {
        const float u = hp_b0_ * v + hp_b1_ * hp_x1_ + hp_b2_ * hp_x2_ - hp_a1_ * hp_y1_ - hp_a2_ * hp_y2_;
        hp_x2_ = hp_x1_; hp_x1_ = v; hp_y2_ = hp_y1_; hp_y1_ = u;
        float yhat = 0.0f;
        for (int k = 0; k < BM_N; ++k) yhat += ws_[k] * osc_s_[k] + wc_[k] * osc_c_[k];
        const float e2 = 2.0f * BM_MU * (u - yhat);
        float vt = 0.0f, vq = 0.0f, at = 0.0f;
        for (int k = 0; k < BM_N; ++k) {
            const float sp = osc_s_[k], cp = osc_c_[k];                          // phase of the current sample
            ws_[k] = (1.0f - BM_LEAK) * ws_[k] + e2 * sp;
            wc_[k] = (1.0f - BM_LEAK) * wc_[k] + e2 * cp;
            // complex amplitude C = wc - j ws, corrected: P = C * K_k (undo HPF + backward difference, +T/2)
            const float pr = wc_[k] * kr_[k] + ws_[k] * ki_[k];
            const float pi = wc_[k] * ki_[k] - ws_[k] * kr_[k];
            const float q = pr * sp + pi * cp;                                   // Im{P e^{j phi}}
            vt += pr * cp - pi * sp;                                             // Re{P e^{j phi}}
            vq += q;
            at -= omega_[k] * q;                                                 // Re{j w P e^{j phi}}
            // advance the oscillator one sample; first-order renormalisation keeps |s + jc| = 1
            const float s = sp * rot_c_[k] + cp * rot_s_[k];
            const float c = cp * rot_c_[k] - sp * rot_s_[k];
            const float g = 1.5f - 0.5f * (s * s + c * c);
            osc_s_[k] = s * g; osc_c_[k] = c * g;
        }
        // Instantaneous envelope = |analytic tremor signal| / sqrt(2) (RMS-equivalent). The sum of squared
        // bin weights is NOT used: with 0.5 Hz spacing and ~100 ms LMS memory the bins are not orthogonal,
        // so individual weights can grow in cancelling directions and overstate power.
        v_tr_ = vt; a_tr_ = at; env_ = sqrtf(0.5f * (vt * vt + vq * vq));
    }

    void bandpass(float v) {
        const float y = bp_b0_ * v + bp_b2_ * bp_x2_ - bp_a1_ * bp_y1_ - bp_a2_ * bp_y2_;
        bp_x2_ = bp_x1_; bp_x1_ = v; bp_y2_ = bp_y1_; bp_y1_ = y;
        v_tr_ = y; a_tr_ = 0.0f;
        bp_ms_ += 0.0198f * (y * y - bp_ms_);                                   // 100 ms
        env_ = sqrtf(bp_ms_);
    }

    void fractional(float v) {
        for (int j = FO_N - 1; j > 0; --j) fo_hist_[j] = fo_hist_[j - 1];
        fo_hist_[0] = v;
        const float r = alpha_ - 1.0f;            // order of the operator applied to velocity
        float w = 1.0f, y = 0.0f, nr = 0.0f, ni = 0.0f;
        for (int j = 0; j < FO_N; ++j) {
            if (j > 0) w *= 1.0f - (r + 1.0f) / (float)j;   // GL weights (-1)^j C(r, j)
            y += w * fo_hist_[j];
            nr += w * fo_cr_[j]; ni += w * fo_sr_[j];
        }
        // normalisation: |operator| = 1 at FO_FREF_HZ (the T^-r factor cancels)
        fo_out_ = y / sqrtf(nr * nr + ni * ni);
    }

    float breathingCue() const {      // -1 .. +1 : +1 = end of inhale (lighter), -1 = end of exhale (heavier)
        const float t = fmodf((float)tick_ * CONTROL_DT_SEC, CUE_PERIOD_S);
        const float PI = 3.14159265f;
        if (t < CUE_INHALE_S) return -cosf(PI * t / CUE_INHALE_S);
        return cosf(PI * (t - CUE_INHALE_S) / (CUE_PERIOD_S - CUE_INHALE_S));
    }

    float uniformPm1() {
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        return (float)(rng_ >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }

    InnovationConfig cfg_;
    ActuatorParams act_;
    bool pc_enabled_, frozen_;
    float frozen_d_;

    int32_t prev_counts_; bool first_;
    float vel_raw_, vel_f_;
    float lp_x1_, lp_x2_, lp_y1_, lp_y2_;
    float hp_b0_, hp_b1_, hp_b2_, hp_a1_, hp_a2_, hp_x1_, hp_x2_, hp_y1_, hp_y2_;
    float bp_b0_, bp_b2_, bp_a1_, bp_a2_, bp_x1_, bp_x2_, bp_y1_, bp_y2_, bp_ms_;
    float ws_[BM_N], wc_[BM_N], osc_s_[BM_N], osc_c_[BM_N], rot_c_[BM_N], rot_s_[BM_N];
    float kr_[BM_N], ki_[BM_N], omega_[BM_N];
    float fo_hist_[FO_N], fo_cr_[FO_N], fo_sr_[FO_N];
    float v_tr_, a_tr_, env_;
    float grip_n_, grip_prev_, dgrip_f_, f0_;
    float m_g_, m_t_, c_gt_, v_g_, v_t_, rho_;
    float d_, tremor_norm_, grip_rel_, boost_;
    float slewed_b_, sm_x1_, sm_x2_, sm_y1_, sm_y2_, b_vol_, b_tr_, d_tr_f_;
    float alpha_, fo_out_, m_virt_;
    float beta_, energy_j_, prev_volts_; bool prev_coast_;
    float force_cmd_, force_lin_;
    uint32_t tick_, rng_;
    EngineFault fault_;
    uint32_t pc_activations_;
};
