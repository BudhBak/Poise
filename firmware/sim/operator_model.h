#pragma once
// Causal visuomotor operator model for the SymbioGrip SIL benchmark.
//
// What the simulated human can use (all causal, delayed and noisy):
//   * vision: carriage/stylus position delayed by tau_v (80-120 ms), plus the wire drawn on the rig, seen
//     only through a forward PREVIEW window ahead of the perceived position (like an eye looking ahead);
//   * proprioception: carriage velocity delayed by 30 ms;
//   * hearing/touch: contact events (buzz) delayed by 60 ms.
// What it can NOT use: instantaneous true lateral error, wall penetration, controller state.
//
// Motor output: longitudinal velocity servo with muscle saturation; transverse equilibrium-point control
// (intrinsic limb stiffness/damping, no delay) with a delayed visual correction; signal-dependent motor
// noise (Harris & Wolpert 1998) and 8-11 Hz physiological tremor that grows with grip co-contraction.
//
// Grip (what the FSR measures):
//   Causal model:   rest + clench to previewed path curvature + startle after perceived contacts + OU stress
//   Null model:     rest + OU stress only (geometry- and performance-independent; sensitivity control)

#include <math.h>
#include <stdint.h>
#include <random>
#include <vector>
#include <algorithm>
#include "plant.h"

enum class GripModel { Causal, Null };

struct OperatorParams {
    // Trial-level (drawn once per run, shared by modes A/B/C)
    float pace_scale       = 1.0f;
    float tremor_scale     = 1.0f;
    float tremor_freq_hz   = 9.5f;
    int   visual_delay     = 50;      // ticks (2 ms)
    // Fixed population parameters (set from Mode A behaviour only; see docs/CALIBRATION.md)
    int   proprio_delay    = 15;      // ticks
    int   audio_delay      = 30;      // ticks
    float v_cruise         = 0.095f;  // m/s
    float b_intrinsic_x    = 12.0f;   // N*s/m intrinsic muscle viscosity about the intended velocity (no delay)
    float k_v              = 15.0f;   // N*s/m delayed (proprioceptive) velocity-correction gain
    float f_max            = 6.0f;    // N muscle saturation
    float curve_slowdown   = 0.8f;
    float startle_slowdown = 0.35f;
    float preview_near     = 0.005f;  // m, start of the visual preview band ahead of the perceived position
    float preview_far      = 0.015f;  // m, end of the preview band
    float k_limb_y         = 60.0f;   // N/m intrinsic lateral stiffness
    float b_limb_y         = 2.6f;    // N*s/m intrinsic lateral damping
    float g_visual         = 8.0f;    // 1/s visual correction gain
    float lead_time        = 0.06f;   // s lateral aim lead
    float tremor_x_n       = 0.15f;   // N RMS axial physiological tremor at rest
    float tremor_y_n       = 0.015f;  // N RMS lateral physiological tremor at rest (normal, not enhanced)
    float cocontraction_k  = 1.0f;    // limb stiffness gain per unit normalized grip (Gribble & Ostry 1998)
    float sdn_base_n       = 0.010f;  // N, signal-independent motor noise
    float sdn_gain         = 0.020f;  // N per N of muscle force (signal-dependent noise, ~2 % CV)
    // Schmidt's law (Schmidt et al. 1979): lateral command variability proportional to commanded lateral
    // speed, sigma_y = schmidt_k * |dy_cmd/dt|. Pre-registered nominal 0.04 s; swept {0, 0.04, 0.08} s.
    float schmidt_k        = 0.04f;   // s
    GripModel grip_model   = GripModel::Causal;
};

class OperatorModel {
public:
    OperatorModel(const OperatorParams& op, const Plant& plant, uint32_t noise_seed)
        : op_(op), plant_(plant), rng_(noise_seed), gauss_(0.0f, 1.0f) {
        const int hist = std::max(op_.visual_delay, std::max(op_.proprio_delay, op_.audio_delay)) + 2;
        hx_.assign(hist, 0.0f); hy_.assign(hist, 0.0f); hvx_.assign(hist, 0.0f); hc_.assign(hist, 0);
        // Physiological tremor: 2nd-order resonator (Q = 4) driven by white noise, unit RMS.
        const float w = 2.0f * 3.14159265f * op_.tremor_freq_hz * CONTROL_DT_SEC;
        const float r = 1.0f - w / (2.0f * 4.0f);
        tr_a1_ = 2.0f * r * cosf(w);
        tr_a2_ = -r * r;
        tr_gain_ = 1.0f;
        tr_gain_ = 1.0f / measureResonatorRms();
    }

    // Grip force the FSR sees, and the 14-bit ADC reading of the divider + follower.
    float gripNewtons() const { return grip_n_; }
    uint16_t gripAdc() {
        const float a = grip_n_ / (grip_n_ + FSR_FORCE_SCALE_N);
        int v = (int)lroundf(a * ADC_FULL_SCALE + 6.0f * gauss_(rng_));
        if (v < 0) v = 0;
        if (v > 16383) v = 16383;
        return (uint16_t)v;
    }

    HandInput tick(int tick_index) {
        const float dt = CONTROL_DT_SEC;
        const float t = tick_index * dt;
        const PlantState& s = plant_.state();

        // ---- history buffers (perception) ----
        const int n = (int)hx_.size();
        hx_[head_] = (float)s.x; hy_[head_] = (float)s.y; hvx_[head_] = (float)s.vx;
        hc_[head_] = s.contact_event ? 1 : 0;
        auto past = [&](const std::vector<float>& h, int d) { return h[(head_ - d + n * 4) % n]; };
        const float x_v  = past(hx_, op_.visual_delay) + 0.0001f * gauss_(rng_);
        const float y_v  = past(hy_, op_.visual_delay) + 0.00005f * gauss_(rng_);
        const float vx_p = past(hvx_, op_.proprio_delay) + 0.002f * gauss_(rng_);
        const bool heard_contact = hc_[(head_ - op_.audio_delay + n * 4) % n] != 0;
        head_ = (head_ + 1) % n;

        // Internal forward model compensates (imperfectly) for the visual delay.
        const float x_pred = x_v + 0.9f * vx_p * (op_.visual_delay * dt);

        // ---- visual preview of upcoming path demand (curvature seen ahead) ----
        const float kw = plant_.waveNumber();
        const float curv_scale = plant_.params().a_wire * kw * kw;
        float d_prev = 0.0f;
        int n_prev = 0;
        for (float sx = op_.preview_near; sx <= op_.preview_far + 1e-6f; sx += 0.001f, ++n_prev) {
            d_prev += fabsf(plant_.wireCurv(x_pred + sx)) / curv_scale;
        }
        d_prev /= (float)n_prev;    // mean previewed curvature, 0..1
        demand_ += (1.0f - expf(-dt / 0.15f)) * (d_prev - demand_);

        // ---- startle after perceived contacts ----
        if (heard_contact) startle_ = std::min(1.5f, startle_ + 1.0f);
        startle_ *= expf(-dt / 0.8f);

        // ---- grip (co-contraction), 60 ms activation dynamics ----
        const float rho_s = expf(-dt / ((op_.grip_model == GripModel::Causal) ? 2.0f : 1.0f));
        const float sig_s = (op_.grip_model == GripModel::Causal) ? 0.6f : 1.4f;
        stress_ = rho_s * stress_ + sig_s * sqrtf(1.0f - rho_s * rho_s) * gauss_(rng_);
        float grip_cmd = 2.0f + stress_;
        if (op_.grip_model == GripModel::Causal) {
            grip_cmd += 3.5f * demand_ + 2.5f * std::min(1.0f, startle_);
        }
        if (grip_cmd < 0.3f) grip_cmd = 0.3f;
        grip_n_ = (tick_index == 0) ? grip_cmd : grip_n_ + (1.0f - expf(-dt / 0.06f)) * (grip_cmd - grip_n_);
        const float grip_norm = std::min(1.0f, std::max(0.0f, (grip_n_ - 2.0f) / 6.0f));

        // ---- longitudinal velocity servo ----
        const float env = 1.0f - expf(-t / 0.16f);
        const float v_des = env * op_.v_cruise * op_.pace_scale / (1.0f + op_.curve_slowdown * demand_)
                          * (1.0f - op_.startle_slowdown * std::min(1.0f, startle_));
        // Intrinsic viscosity acts on the true limb velocity without delay (muscle force-velocity property);
        // the reflex/visual correction acts on delayed proprioceptive velocity.
        const float f_reach = op_.f_max * tanhf((op_.b_intrinsic_x * (v_des - (float)s.vx)
                                                 + op_.k_v * (v_des - vx_p)) / op_.f_max);

        // ---- noise: signal-dependent (AR(1), tau = 15 ms) + physiological tremor ----
        const float rho_n = expf(-dt / 0.015f);
        const float sig_x = op_.sdn_base_n + op_.sdn_gain * fabsf(f_reach);
        nx_ = rho_n * nx_ + sig_x * sqrtf(1.0f - rho_n * rho_n) * gauss_(rng_);
        const float trem_amp = op_.tremor_scale * (1.0f + 1.5f * grip_norm);
        const float trem_x = op_.tremor_x_n * trem_amp * resonate(trx_);
        const float trem_y = op_.tremor_y_n * trem_amp * resonate(try_);

        // ---- lateral: equilibrium point = previewed wire ahead + delayed visual correction ----
        float lead = std::min(0.010f, std::max(0.0f, vx_p * op_.lead_time));
        const float aim = plant_.wireY(x_pred + lead);
        corr_ += dt * (op_.g_visual * (plant_.wireY(x_v) - y_v) - 0.5f * corr_);
        // Speed-dependent command noise (AR(1), tau = 50 ms) scaled by the commanded lateral speed.
        // Intended lateral command speed = path slope at the aim point x perceived carriage speed.
        const float aim_rate = plant_.wireSlope(x_pred + lead) * vx_p;
        aim_rate_f_ += (1.0f - expf(-dt / 0.05f)) * (fabsf(aim_rate) - aim_rate_f_);
        const float rho_c = expf(-dt / 0.05f);
        ncmd_ = rho_c * ncmd_ + op_.schmidt_k * aim_rate_f_ * sqrtf(1.0f - rho_c * rho_c) * gauss_(rng_);
        const float y_eq = aim + corr_ + ncmd_;
        const float f_lat_est = fabsf(op_.k_limb_y * (y_eq - y_v));
        const float sig_y = 0.004f + op_.sdn_gain * f_lat_est;
        ny_ = rho_n * ny_ + sig_y * sqrtf(1.0f - rho_n * rho_n) * gauss_(rng_);

        // Co-contraction stiffens the limb (damping scaled to keep the damping ratio constant).
        const float stiff = 1.0f + op_.cocontraction_k * grip_norm;
        HandInput h;
        h.fx = f_reach + nx_ + trem_x;
        h.y_eq = y_eq; h.ky = op_.k_limb_y * stiff; h.by = op_.b_limb_y * sqrtf(stiff);
        h.fy = ny_ + trem_y;
        return h;
    }

private:
    struct Res { float y1 = 0.0f, y2 = 0.0f; };
    float resonate(Res& r) {
        const float y = tr_a1_ * r.y1 + tr_a2_ * r.y2 + tr_gain_ * gauss_(rng_);
        r.y2 = r.y1; r.y1 = y;
        return y;
    }
    float measureResonatorRms() {
        std::mt19937 tmp(12345);
        std::normal_distribution<float> g(0.0f, 1.0f);
        float y1 = 0, y2 = 0; double acc = 0; const int N = 40000;
        for (int i = 0; i < N; ++i) {
            const float y = tr_a1_ * y1 + tr_a2_ * y2 + g(tmp);
            y2 = y1; y1 = y;
            if (i >= 2000) acc += (double)y * y;
        }
        return (float)sqrt(acc / (N - 2000));
    }

    OperatorParams op_;
    const Plant& plant_;
    std::mt19937 rng_;
    std::normal_distribution<float> gauss_;
    std::vector<float> hx_, hy_, hvx_;
    std::vector<uint8_t> hc_;
    int head_ = 0;
    float demand_ = 0.0f, startle_ = 0.0f, stress_ = 0.0f, grip_n_ = 2.0f;
    float nx_ = 0.0f, ny_ = 0.0f, corr_ = 0.0f;
    float aim_rate_f_ = 0.0f, ncmd_ = 0.0f;
    float tr_a1_, tr_a2_, tr_gain_;
    Res trx_, try_;
};
