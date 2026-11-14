// SymbioGrip v3 innovation lab: candidate controllers vs. the fixed-damper family and v2 Mode C.
//
// Shares plant.h and operator_model.h (frozen) with test_bench.cpp and reproduces its per-seed operator draws
// and common-random-number scheme exactly, so lab baselines equal the committed SIL data for the same seed.
//
// Usage:
//   innovation_lab --suite S1|S2a|S2b|S3|S4|S5|D0|CAL|PARITY|ADV --out DIR [--runs 10] [--seed 2026]
//
//   S1   primary causal operator (Schmidt k = 0.04 s)
//   S2a  high command noise (k = 0.08 s)              S2b  enhanced physiological tremor (x4 force, x and y)
//   S3   confident firm grip: FSR reads a steady 8.0 N, operator tremor scale 0.5
//   S4   cross-user resting grip 1.5 / 3.0 / 4.5 / 6.0 N (sensor-level offset), sub-folders rest_<N>
//   S5   breathing-cue / dither non-interference (DB_ADAPT vs +swell vs +dither)
//   D0   structural diagnostics in Mode A: x-tremor removed, all x-noise removed (upper bound of x-only benefit)
//   CAL  calibration data (development seed only): quantisation-only glides + Mode A tremor envelopes
//   PARITY  InnovationEngine with fixed B = 13 vs HapticEngine mode B (must be identical)
//   ADV  adversarial passivity, quantisation, envelope latency, linearity and frequency responses -> adv.json

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "../src/admittance.h"
#include "innovation_engine.h"
#include "operator_model.h"
#include "plant.h"

namespace fs = std::filesystem;

// ------------------------------------------------------------------------------------------------
// Controller abstraction
// ------------------------------------------------------------------------------------------------
struct Ctrl {
    virtual ~Ctrl() {}
    virtual MotorCommand step(uint16_t adc, int32_t counts) = 0;
    virtual float velocity() const = 0;
    virtual float damping() const = 0;
    virtual float difficulty() const = 0;
    virtual float tremor() const = 0;
    virtual float energy() const = 0;
    virtual float beta() const = 0;
    virtual uint32_t pc() const = 0;
    virtual uint8_t fault() const = 0;
    virtual void extras(float* e) const { for (int i = 0; i < 7; ++i) e[i] = 0.0f; }
    virtual void setPc(bool on) = 0;
};

struct V2Ctrl : Ctrl {
    HapticEngine e;
    V2Ctrl(char mode, float b, int32_t counts, const ActuatorParams& a) : e(a) {
        if (mode == 'B') e.setFixedDamping(b);
        e.setMode(mode, counts);
    }
    MotorCommand step(uint16_t adc, int32_t c) override { return e.step(adc, c); }
    float velocity() const override { return e.velocity(); }
    float damping() const override { return e.damping(); }
    float difficulty() const override { return e.difficulty(); }
    float tremor() const override { return e.tremorRms(); }
    float energy() const override { return e.observerEnergy(); }
    float beta() const override { return e.passivityBeta(); }
    uint32_t pc() const override { return e.pcActivations(); }
    uint8_t fault() const override { return e.fault(); }
    void setPc(bool on) override { e.setPassivityControllerEnabled(on); }
};

struct V3Ctrl : Ctrl {
    InnovationEngine e;
    V3Ctrl(const InnovationConfig& c, int32_t counts, const ActuatorParams& a) : e(c, a) { e.reset(counts); }
    MotorCommand step(uint16_t adc, int32_t c) override { return e.step(adc, c); }
    float velocity() const override { return e.velocity(); }
    float damping() const override { return e.damping(); }
    float difficulty() const override { return e.difficulty(); }
    float tremor() const override { return e.tremorEnvelope(); }
    float energy() const override { return e.observerEnergy(); }
    float beta() const override { return e.passivityBeta(); }
    uint32_t pc() const override { return e.pcActivations(); }
    uint8_t fault() const override { return e.fault(); }
    void extras(float* x) const override {
        x[0] = e.tremorDamping(); x[1] = e.tremorVelocity(); x[2] = e.tremorEnvelope(); x[3] = e.rho();
        x[4] = e.baseline(); x[5] = e.virtualMass(); x[6] = e.alpha();
    }
    void setPc(bool on) override { e.setPassivityControllerEnabled(on); }
};

struct Condition {
    std::string name;
    bool v2;
    char mode;          // v2 only
    float b;            // v2 mode B damping
    InnovationConfig cfg;
};

static InnovationConfig dbBase() {
    InnovationConfig c; c.tremor_channel = 1; c.b_vol_lo = c.b_vol_hi = 3.75f; c.b_tr_lo = c.b_tr_hi = 33.5f;
    return c;
}
static InnovationConfig adaptBase() {
    InnovationConfig c = dbBase();
    c.b_vol_lo = 2.5f; c.b_vol_hi = 5.0f; c.b_tr_lo = 22.0f; c.b_tr_hi = 45.0f;
    c.f0_tracking = c.anticipator = c.rho_gate = true;
    return c;
}

static std::vector<Condition> baselineConditions() {
    return {
        {"A", true, 'A', 0.0f, {}},     {"B4.5", true, 'B', 4.5f, {}},   {"B8.22", true, 'B', 8.22f, {}},
        {"B13", true, 'B', 13.0f, {}},  {"B20", true, 'B', 20.0f, {}},   {"C_v2", true, 'C', 0.0f, {}},
    };
}

static std::vector<Condition> candidateConditions() {
    std::vector<Condition> v;
    v.push_back({"DB_BMFLC", false, 0, 0, dbBase()});
    { InnovationConfig c = dbBase(); c.tremor_channel = 2; v.push_back({"DB_LTI", false, 0, 0, c}); }
    v.push_back({"DB_ADAPT", false, 0, 0, adaptBase()});
    { InnovationConfig c; c.fractional = true; c.b_vol_lo = c.b_vol_hi = 3.75f; c.tremor_channel = 1; c.tremor_force = false;
      c.fo_alpha_lo = 0.55f; c.fo_alpha_hi = 0.95f; c.f0_tracking = c.rho_gate = true;
      v.push_back({"FO_SUB", false, 0, 0, c}); }
    { InnovationConfig c; c.fractional = true; c.b_vol_lo = c.b_vol_hi = 3.75f; c.tremor_channel = 1; c.tremor_force = false;
      c.fo_alpha_lo = 1.05f; c.fo_alpha_hi = 1.45f; c.f0_tracking = c.rho_gate = true;
      v.push_back({"FO_SUPER", false, 0, 0, c}); }
    { InnovationConfig c = dbBase(); c.m_virt_max = 1.0f; v.push_back({"DB_VI", false, 0, 0, c}); }
    { InnovationConfig c = adaptBase(); c.m_virt_max = 1.0f; v.push_back({"FULL_NODS", false, 0, 0, c}); }
    { InnovationConfig c = adaptBase(); c.m_virt_max = 1.0f; c.dither_duty = 0.025f; c.swell_amp = 0.2f;
      v.push_back({"FULL", false, 0, 0, c}); }
    { InnovationConfig c; c.tremor_channel = 1; c.tremor_force = false; c.b_vol_lo = DAMPING_MIN; c.b_vol_hi = DAMPING_MAX;
      c.f0_tracking = c.anticipator = c.rho_gate = true; v.push_back({"SCALAR_F0", false, 0, 0, c}); }
    return v;
}

static std::unique_ptr<Ctrl> make(const Condition& c, int32_t counts, const ActuatorParams& a = defaultActuatorParams()) {
    if (c.v2) return std::unique_ptr<Ctrl>(new V2Ctrl(c.mode, c.b, counts, a));
    return std::unique_ptr<Ctrl>(new V3Ctrl(c.cfg, counts, a));
}

// ------------------------------------------------------------------------------------------------
// Trials
// ------------------------------------------------------------------------------------------------
struct SuiteSpec {
    std::string name;
    float schmidt_k = 0.04f;
    float tremor_force_scale = 1.0f;     // multiplies tremor_x_n and tremor_y_n
    float fixed_tremor_scale = -1.0f;    // >= 0 overrides the per-seed tremor_scale draw
    float sensor_grip_const = -1.0f;     // >= 0: FSR reads this force (N) instead of the operator's grip
    float sensor_grip_offset = 0.0f;     // added to the operator's grip at the sensor
    bool zero_x_tremor = false, zero_x_noise = false;
};

static uint16_t forceToAdc(float f, float noise) {
    if (f < 0.0f) f = 0.0f;
    const float a = f / (f + FSR_FORCE_SCALE_N);
    int v = (int)lroundf(a * ADC_FULL_SCALE + noise);
    return (uint16_t)std::min(16383, std::max(0, v));
}

static const char* HEADER =
    "t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts,"
    "pos_m,y_m,lat_err_m,grip_true_N,motor_force_N,motor_current_A,pc_beta,"
    "b_tr,v_tr,tr_env,rho_gt,grip_f0,m_virt,alpha\n";

static void runSuite(const SuiteSpec& sp, const std::vector<Condition>& conds, const fs::path& out, int runs, uint32_t seed) {
    fs::create_directories(out);
    PlantParams pp;
    std::ostringstream man;
    man << "{\n  \"generator\": \"firmware/sim/innovation_lab.cpp\",\n  \"suite\": \"" << sp.name << "\",\n  \"seed\": " << seed
        << ", \"runs\": " << runs << ",\n  \"schmidt_k_s\": " << sp.schmidt_k << ", \"tremor_force_scale\": " << sp.tremor_force_scale
        << ", \"fixed_tremor_scale\": " << sp.fixed_tremor_scale << ", \"sensor_grip_const\": " << sp.sensor_grip_const
        << ", \"sensor_grip_offset\": " << sp.sensor_grip_offset << ",\n  \"counts_per_meter\": " << COUNTS_PER_METER
        << ",\n  \"plant\": {\"track_len_m\": " << pp.track_len << ", \"corridor_half_width_m\": " << pp.corridor_r
        << ", \"wire_amplitude_m\": " << pp.a_wire << ", \"wire_wavelength_m\": " << pp.lambda_wire << "},\n  \"trials\": [\n";
    bool first_row = true;
    const int MAX_TICKS = 7500;
    for (int run = 1; run <= runs; ++run) {
        // identical draws to test_bench.cpp
        std::mt19937 trial_rng(seed * 7919u + (uint32_t)run);
        std::normal_distribution<float> g(0.0f, 1.0f);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        OperatorParams op;
        op.grip_model = GripModel::Causal;
        op.pace_scale = std::min(1.25f, std::max(0.75f, 1.0f + 0.08f * g(trial_rng)));
        op.tremor_scale = std::min(1.6f, std::max(0.5f, 1.0f + 0.20f * g(trial_rng)));
        op.tremor_freq_hz = 8.0f + 3.0f * u(trial_rng);
        op.visual_delay = std::min(60, std::max(40, 50 + (int)std::lround(4.0f * g(trial_rng))));
        op.schmidt_k = sp.schmidt_k;
        op.tremor_x_n *= sp.tremor_force_scale; op.tremor_y_n *= sp.tremor_force_scale;
        if (sp.fixed_tremor_scale >= 0.0f) op.tremor_scale = sp.fixed_tremor_scale;
        if (sp.zero_x_tremor) op.tremor_x_n = 0.0f;
        if (sp.zero_x_noise) { op.tremor_x_n = 0.0f; op.sdn_base_n = 0.0f; }
        const uint32_t noise_seed = seed * 104729u + (uint32_t)run * 31u;

        for (const Condition& c : conds) {
            Plant plant(pp);
            OperatorParams opc = op;
            OperatorModel human(opc, plant, noise_seed);
            std::mt19937 sensor_rng(noise_seed ^ 0x9E3779B9u);
            std::normal_distribution<float> sg(0.0f, 1.0f);
            auto ctl = make(c, plant.encoderCounts());
            std::ofstream f(out / ("trial_" + c.name + "_run_" + std::to_string(run) + ".csv"));
            f << HEADER;
            char line[640];
            float emin = 1e9f, ex[7];
            int tick = 0;
            for (; tick < MAX_TICKS && !plant.finished(); ++tick) {
                uint16_t adc = human.gripAdc();     // always called: keeps the operator's RNG stream identical
                if (sp.sensor_grip_const >= 0.0f) adc = forceToAdc(sp.sensor_grip_const + 0.05f * sg(sensor_rng), 6.0f * sg(sensor_rng));
                else if (sp.sensor_grip_offset != 0.0f) adc = forceToAdc(human.gripNewtons() + sp.sensor_grip_offset, 6.0f * sg(sensor_rng));
                const int32_t counts = plant.encoderCounts();
                const MotorCommand cmd = ctl->step(adc, counts);
                const HandInput hand = human.tick(tick);
                plant.advance(hand, cmd);
                emin = std::min(emin, ctl->energy());
                if (tick % 5 == 0 || plant.finished()) {
                    const PlantState& s = plant.state();
                    ctl->extras(ex);
                    std::snprintf(line, sizeof line,
                        "%d,%s,%u,%ld,%.5f,%.3f,%.4f,%.5f,%.4f,%.4f,%d,%.6f,%.7f,%.7f,%.4f,%.4f,%.4f,%.3f,%.3f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f\n",
                        (tick + 1) * 2, c.name.c_str(), (unsigned)adc, (long)counts, ctl->velocity(), ctl->damping(),
                        ctl->difficulty(), ctl->tremor(), cmd.coast ? 0.0f : cmd.duty, ctl->energy() * 1000.0f, s.contacts,
                        s.x, s.y, plant.lateralError(), human.gripNewtons(), s.motor_force, s.motor_current, ctl->beta(),
                        ex[0], ex[1], ex[2], ex[3], ex[4], ex[5], ex[6]);
                    f << line;
                }
            }
            man << (first_row ? "" : ",\n") << "    {\"run\": " << run << ", \"cond\": \"" << c.name << "\", \"completed\": "
                << (plant.finished() ? "true" : "false") << ", \"duration_s\": " << tick * CONTROL_DT_SEC
                << ", \"contacts\": " << plant.state().contacts << ", \"max_current_a\": " << plant.state().max_abs_current
                << ", \"motor_work_j\": " << plant.state().motor_work_j << ", \"min_observer_j\": " << emin
                << ", \"pc_activations\": " << ctl->pc() << ", \"fault\": " << (int)ctl->fault() << "}";
            first_row = false;
        }
    }
    man << "\n  ]\n}\n";
    std::ofstream(out / "manifest.json") << man.str();
    std::cout << "[lab] " << sp.name << ": " << runs << " seeds x " << conds.size() << " conditions -> " << out.string() << "\n";
}

// ------------------------------------------------------------------------------------------------
// Adversarial / verification suite
// ------------------------------------------------------------------------------------------------
struct ClosedLoop { double work; float emin; uint32_t pc; float vmax; };

static ClosedLoop closedLoop(const Condition& c, const ActuatorParams& a, bool pc_on, double seconds) {
    Plant plant;
    auto ctl = make(c, plant.encoderCounts(), a);
    ctl->setPc(pc_on);
    std::mt19937 rng(7);
    std::normal_distribution<float> g(0.0f, 1.0f);
    ClosedLoop r{0.0, 1e9f, 0, 0.0f};
    const int n = (int)(seconds / CONTROL_DT_SEC);
    for (int k = 0; k < n; ++k) {
        const double t = k * CONTROL_DT_SEC;
        const uint16_t grip = (uint16_t)(16383 * (((k / 400) % 2) ? 0.70 : 0.80));
        const MotorCommand cmd = ctl->step(grip, plant.encoderCounts());
        HandInput h;
        h.x_eq = (float)(0.09 + 0.05 * sin(2 * M_PI * 0.4 * t)); h.kx = 50.0f; h.bx = 2.0f;
        h.fx = (float)(0.4 * sin(2 * M_PI * 9.0 * t)) + 0.05f * g(rng);
        h.ky = 60.0f; h.by = 2.6f;
        plant.advance(h, cmd);
        r.emin = std::min(r.emin, ctl->energy());
        r.vmax = std::max(r.vmax, (float)fabs(plant.state().vx));
    }
    r.work = plant.state().motor_work_j; r.pc = ctl->pc();
    return r;
}

static ClosedLoop reversals(const Condition& c, const ActuatorParams& a, double amp_m, double hz, double seconds) {
    Plant plant;
    plant.prescribe(0.09, MotorCommand{0.0f, true});
    auto ctl = make(c, plant.encoderCounts(), a);
    ClosedLoop r{0.0, 1e9f, 0, 0.0f};
    const double w0 = plant.state().motor_work_j;
    const int n = (int)(seconds / CONTROL_DT_SEC);
    for (int k = 0; k < n; ++k) {
        const double t = (k + 1) * CONTROL_DT_SEC;
        const MotorCommand cmd = ctl->step((uint16_t)(16383 * 0.75), plant.encoderCounts());
        plant.prescribe(0.09 + amp_m * sin(2 * M_PI * hz * t), cmd);
        r.emin = std::min(r.emin, ctl->energy());
    }
    r.work = plant.state().motor_work_j - w0; r.pc = ctl->pc(); r.vmax = (float)(2 * M_PI * hz * amp_m);
    return r;
}

struct QuantResult { float max_env; float max_d; float frac_d; };
static QuantResult quantGlides(const Condition& c) {
    QuantResult q{0.0f, 0.0f, 0.0f};
    long n_all = 0, n_on = 0;
    for (double v : {0.01, 0.03, 0.06, 0.10, 0.20}) {
        auto ctl = make(c, 0);
        for (int k = 0; k < 6000; ++k) {
            const double t = k * CONTROL_DT_SEC;
            const double x = v * t + 0.0003 * sin(2 * M_PI * 0.37 * t);
            ctl->step(forceToAdc(2.0f, 0.0f), (int32_t)floor(x * COUNTS_PER_METER));
            if (k > 1000) {
                q.max_env = std::max(q.max_env, ctl->tremor());
                q.max_d = std::max(q.max_d, ctl->difficulty());
                ++n_all; if (ctl->difficulty() > 0.05f) ++n_on;
            }
        }
    }
    q.frac_d = (float)n_on / (float)n_all;
    return q;
}

// Envelope latency: tremor burst (9 Hz, 6 mm/s velocity amplitude) at t = 1 s on a 80 mm/s glide.
static float latency63(const Condition& c) {
    auto ctl = make(c, 0);
    const int n = 2500, on = 500;
    std::vector<float> env(n);
    const double A = 0.006 / (2 * M_PI * 9.0);
    for (int k = 0; k < n; ++k) {
        const double t = k * CONTROL_DT_SEC;
        double x = 0.08 * t;
        if (k >= on) x += A * sin(2 * M_PI * 9.0 * (t - on * CONTROL_DT_SEC));
        ctl->step(forceToAdc(2.0f, 0.0f), (int32_t)floor(x * COUNTS_PER_METER));
        env[k] = ctl->tremor();
    }
    double pre = 0, post = 0;
    for (int k = on - 250; k < on; ++k) pre += env[k];
    for (int k = 2000; k < 2500; ++k) post += env[k];
    pre /= 250; post /= 500;
    const double thr = pre + 0.632 * (post - pre);
    for (int k = on; k < n; ++k) if (env[k] >= thr) return (float)((k - on) * CONTROL_DT_SEC);
    return -1.0f;
}

// Frequency response of the linear force path (difficulty frozen), exact increments (no quantisation).
static std::vector<std::complex<double>> freqResponse(const InnovationConfig& cfg, float d, const std::vector<double>& f_hz) {
    std::vector<std::complex<double>> G;
    for (double f : f_hz) {
        InnovationEngine e(cfg);
        e.freezeDifficulty(true, d);
        e.reset(0);
        const double w = 2 * M_PI * f * CONTROL_DT_SEC;
        const int period = (int)std::ceil(2 * M_PI / w);
        const int settle = 4000, meas = std::max(4000, period * 3);
        std::complex<double> sf(0, 0), sv(0, 0);
        const double amp = 0.01;
        for (int k = 0; k < settle + meas; ++k) {
            const double v = amp * cos(w * k);
            e.stepCore((float)(v * CONTROL_DT_SEC), forceToAdc(2.0f, 0.0f));
            if (k >= settle) {
                const std::complex<double> ph = std::polar(1.0, -w * k);
                sf += (double)(-e.forceLinear()) * ph;      // G = -F / v  (positive = damping)
                sv += v * ph;
            }
        }
        G.push_back(sf / sv);
    }
    return G;
}

static double linearityError(const InnovationConfig& cfg) {
    auto run = [&](int which) {
        InnovationEngine e(cfg); e.freezeDifficulty(true, 0.5f); e.reset(0);
        std::vector<double> out;
        for (int k = 0; k < 6000; ++k) {
            const double t = k * CONTROL_DT_SEC;
            const double v1 = 0.004 * sin(2 * M_PI * 7.0 * t);
            const double v2 = 0.003 * sin(2 * M_PI * 11.3 * t + 0.4) + 0.05 * sin(2 * M_PI * 1.7 * t);
            const double v = (which == 1 ? v1 : which == 2 ? v2 : v1 + v2);
            e.stepCore((float)(v * CONTROL_DT_SEC), forceToAdc(2.0f, 0.0f));
            out.push_back(e.forceLinear());
        }
        return out;
    };
    auto a = run(1), b = run(2), ab = run(3);
    double emax = 0, ref = 0;
    for (size_t i = 0; i < a.size(); ++i) { emax = std::max(emax, fabs(ab[i] - a[i] - b[i])); ref = std::max(ref, fabs(ab[i])); }
    return emax / ref;
}

static void runAdversarial(const fs::path& out) {
    fs::create_directories(out);
    std::vector<Condition> conds = candidateConditions();
    conds.insert(conds.begin(), Condition{"B13", true, 'B', 13.0f, {}});
    conds.insert(conds.begin() + 1, Condition{"C_v2", true, 'C', 0.0f, {}});
    ActuatorParams nom = defaultActuatorParams();
    ActuatorParams over = nom; over.bemf_comp = 2.5f;
    std::vector<double> fgrid;
    for (int i = 0; i < 320; ++i) fgrid.push_back(0.2 * pow(250.0 / 0.2, i / 319.0));
    std::ofstream js(out / "adv.json");
    js.precision(9);
    js << "{\n  \"freq_hz\": [";
    for (size_t i = 0; i < fgrid.size(); ++i) js << (i ? "," : "") << fgrid[i];
    js << "],\n  \"conditions\": {\n";
    for (size_t ci = 0; ci < conds.size(); ++ci) {
        const Condition& c = conds[ci];
        const ClosedLoop nomr = closedLoop(c, nom, true, 20.0);
        const ClosedLoop off = closedLoop(c, over, false, 20.0);
        const ClosedLoop onr = closedLoop(c, over, true, 20.0);
        const ClosedLoop rn = reversals(c, nom, 0.004, 8.0, 4.0);
        const ClosedLoop ro = reversals(c, over, 0.004, 8.0, 4.0);
        const QuantResult q = quantGlides(c);
        const float lat = latency63(c);
        js << (ci ? ",\n" : "") << "    \"" << c.name << "\": {\"v2\": " << (c.v2 ? "true" : "false")
           << ", \"nominal\": {\"work_j\": " << nomr.work << ", \"emin_j\": " << nomr.emin << ", \"pc\": " << nomr.pc << "}"
           << ", \"overcomp_pc_off\": {\"work_j\": " << off.work << ", \"vmax\": " << off.vmax << "}"
           << ", \"overcomp_pc_on\": {\"work_j\": " << onr.work << ", \"emin_j\": " << onr.emin << ", \"pc\": " << onr.pc << "}"
           << ", \"reversal8hz_nominal\": {\"work_j\": " << rn.work << ", \"emin_j\": " << rn.emin << ", \"pc\": " << rn.pc << "}"
           << ", \"reversal8hz_overcomp\": {\"work_j\": " << ro.work << ", \"emin_j\": " << ro.emin << ", \"pc\": " << ro.pc << "}"
           << ", \"quant\": {\"max_env\": " << q.max_env << ", \"max_d\": " << q.max_d << ", \"frac_d_gt_0.05\": " << q.frac_d << "}"
           << ", \"latency63_s\": " << lat;
        if (!c.v2) {
            js << ", \"bemf_comp\": " << nom.bemf_comp << ", \"linearity_rel_err\": " << linearityError(c.cfg);
            for (float d : {0.0f, 1.0f}) {
                auto G = freqResponse(c.cfg, d, fgrid);
                js << ", \"G_d" << (int)d << "_re\": [";
                for (size_t i = 0; i < G.size(); ++i) js << (i ? "," : "") << G[i].real();
                js << "], \"G_d" << (int)d << "_im\": [";
                for (size_t i = 0; i < G.size(); ++i) js << (i ? "," : "") << G[i].imag();
                js << "]";
            }
        }
        js << "}";
        std::cout << "[adv] " << c.name << " done\n";
    }
    js << "\n  }\n}\n";
}

// ------------------------------------------------------------------------------------------------
static void runParity(const fs::path& out, int runs, uint32_t seed) {
    InnovationConfig c; c.b_vol_lo = c.b_vol_hi = 13.0f;
    std::vector<Condition> conds = {{"B13", true, 'B', 13.0f, {}}, {"V3_FIXED13", false, 0, 0, c}};
    SuiteSpec sp; sp.name = "PARITY";
    runSuite(sp, conds, out, runs, seed);
}

int main(int argc, char** argv) {
    std::string suite, out;
    int runs = 10;
    uint32_t seed = 2026;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() { if (i + 1 >= argc) { std::exit(2); } return std::string(argv[++i]); };
        if (a == "--suite") suite = next();
        else if (a == "--out") out = next();
        else if (a == "--runs") runs = std::atoi(next().c_str());
        else if (a == "--seed") seed = (uint32_t)std::strtoul(next().c_str(), nullptr, 10);
        else { std::cerr << "unknown arg " << a << "\n"; return 2; }
    }
    if (suite.empty() || out.empty()) {
        std::cerr << "usage: innovation_lab --suite S1|S2a|S2b|S3|S4|S5|D0|CAL|PARITY|ADV --out DIR [--runs N] [--seed S]\n";
        return 2;
    }
    std::vector<Condition> all = baselineConditions();
    for (auto& c : candidateConditions()) all.push_back(c);
    const fs::path o(out);
    SuiteSpec sp; sp.name = suite;
    if (suite == "S1") runSuite(sp, all, o, runs, seed);
    else if (suite == "S2a") { sp.schmidt_k = 0.08f; runSuite(sp, all, o, runs, seed); }
    else if (suite == "S2b") { sp.tremor_force_scale = 4.0f; runSuite(sp, all, o, runs, seed); }
    else if (suite == "S3") { sp.sensor_grip_const = 8.0f; sp.fixed_tremor_scale = 0.5f; runSuite(sp, all, o, runs, seed); }
    else if (suite == "S4") {
        for (float rest : {1.5f, 3.0f, 4.5f, 6.0f}) {
            SuiteSpec s4 = sp; s4.sensor_grip_offset = rest - 2.0f;
            char sub[32]; std::snprintf(sub, sizeof sub, "rest_%.1f", rest);
            s4.name = std::string("S4_") + sub;
            runSuite(s4, all, o / sub, runs, seed);
        }
    } else if (suite == "S5") {
        std::vector<Condition> c5 = baselineConditions();
        InnovationConfig a = adaptBase();
        c5.push_back({"DB_ADAPT", false, 0, 0, a});
        InnovationConfig s = a; s.swell_amp = 0.2f; c5.push_back({"DB_ADAPT_SWELL", false, 0, 0, s});
        InnovationConfig d = a; d.dither_duty = 0.025f; c5.push_back({"DB_ADAPT_DITHER", false, 0, 0, d});
        runSuite(sp, c5, o, runs, seed);
    } else if (suite == "D0") {
        std::vector<Condition> ca = {{"A", true, 'A', 0.0f, {}}};
        runSuite(sp, ca, o / "nominal", runs, seed);
        SuiteSpec z = sp; z.zero_x_tremor = true; z.name = "D0_no_x_tremor"; runSuite(z, ca, o / "no_x_tremor", runs, seed);
        SuiteSpec zz = sp; zz.zero_x_noise = true; zz.name = "D0_no_x_tremor_no_base_noise"; runSuite(zz, ca, o / "no_x_tremor_no_base_noise", runs, seed);
    } else if (suite == "CAL") {
        // development seed only: Mode A operator with a sensing-only BMFLC to record tremor envelopes
        InnovationConfig s; s.tremor_channel = 1; s.tremor_force = false; s.actuate = false; s.b_vol_lo = s.b_vol_hi = 0.0f;
        std::vector<Condition> cc = {{"SENSE_ONLY_B0", false, 0, 0, s}};
        runSuite(sp, cc, o, runs, seed);
        std::ofstream q(o / "quant.json");
        const QuantResult r = quantGlides(cc[0]);
        q << "{\"max_env\": " << r.max_env << ", \"max_d\": " << r.max_d << "}\n";
        std::cout << "[cal] quantisation-only max envelope = " << r.max_env << " m/s\n";
    } else if (suite == "PARITY") runParity(o, runs, seed);
    else if (suite == "ADV") runAdversarial(o);
    else { std::cerr << "unknown suite\n"; return 2; }
    return 0;
}
