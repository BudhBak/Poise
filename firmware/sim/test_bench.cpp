// SymbioGrip multi-rate SIL benchmark: 4 kHz plant / 500 Hz firmware controller.
//
// The controller under test is the unmodified firmware HapticEngine::step(): it receives the 14-bit grip
// ADC reading and floor-quantised encoder counts, and returns the TB6612 duty/coast command that the
// plant's motor model turns into force (back-EMF, current, friction included).
//
// Usage:
//   test_bench --out <dir> [--grip causal|null] [--runs 10] [--seed 2026] [--modes ABC] [--log-div 5]
//
// Fairness: per run, all trial-level parameters AND the per-tick noise stream (common random numbers)
// are identical across modes A, B and C. Only the controller mode differs.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include "../src/admittance.h"
#include "plant.h"
#include "operator_model.h"

namespace fs = std::filesystem;

struct TrialSummary {
    int run; char mode; bool completed; double duration_s; int contacts;
    float max_current_a; uint32_t pc_activations; int fault;
};

static void usage() {
    std::cerr << "usage: test_bench --out <dir> [--grip causal|null] [--runs N] [--seed S] [--modes ABC] [--log-div K]\n"
                 "                  [--schmidt-k SECONDS] [--pace SCALE]\n";
}

int main(int argc, char** argv) {
    std::string out_dir, modes = "ABC", grip = "causal";
    int runs = 10, log_div = 5;
    uint32_t seed = 2026;
    float schmidt_k = OperatorParams().schmidt_k;
    float b_fixed = DAMPING_FIXED;  // mode B damping; override only for the dose-matched sensitivity run
    float pace_override = 0.0f;   // > 0: fixed pace scale for all runs (Mode A speed-accuracy calibration only)
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { if (i + 1 >= argc) { usage(); std::exit(2); } return argv[++i]; };
        if (a == "--out") out_dir = next();
        else if (a == "--grip") grip = next();
        else if (a == "--runs") runs = std::atoi(next().c_str());
        else if (a == "--seed") seed = (uint32_t)std::strtoul(next().c_str(), nullptr, 10);
        else if (a == "--modes") modes = next();
        else if (a == "--log-div") log_div = std::atoi(next().c_str());
        else if (a == "--schmidt-k") schmidt_k = (float)std::atof(next().c_str());
        else if (a == "--pace") pace_override = (float)std::atof(next().c_str());
        else if (a == "--b-fixed") b_fixed = (float)std::atof(next().c_str());
        else { usage(); return 2; }
    }
    if (out_dir.empty() || (grip != "causal" && grip != "null") || runs < 1 || log_div < 1) { usage(); return 2; }
    fs::create_directories(out_dir);

    const GripModel gm = (grip == "causal") ? GripModel::Causal : GripModel::Null;
    const int MAX_TICKS = 7500;  // 15 s timeout
    PlantParams pp;
    std::vector<TrialSummary> summaries;

    std::cout << "SymbioGrip SIL: 4 kHz plant / 500 Hz firmware step(), grip model = " << grip
              << ", runs = " << runs << ", seed = " << seed << "\n";

    for (int run = 1; run <= runs; ++run) {
        std::mt19937 trial_rng(seed * 7919u + (uint32_t)run);
        std::normal_distribution<float> g(0.0f, 1.0f);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        OperatorParams op;
        op.grip_model = gm;
        op.pace_scale = std::min(1.25f, std::max(0.75f, 1.0f + 0.08f * g(trial_rng)));
        op.tremor_scale = std::min(1.6f, std::max(0.5f, 1.0f + 0.20f * g(trial_rng)));
        op.tremor_freq_hz = 8.0f + 3.0f * u(trial_rng);
        op.visual_delay = std::min(60, std::max(40, 50 + (int)std::lround(4.0f * g(trial_rng))));
        op.schmidt_k = schmidt_k;
        if (pace_override > 0.0f) op.pace_scale = pace_override;
        const uint32_t noise_seed = seed * 104729u + (uint32_t)run * 31u;   // identical for A, B, C

        for (char mode : modes) {
            Plant plant(pp);
            OperatorModel human(op, plant, noise_seed);
            HapticEngine engine;
            engine.setFixedDamping(b_fixed);
            engine.setMode(mode, plant.encoderCounts());

            std::ostringstream name;
            name << "trial_mode_" << mode << "_run_" << run << "_sil.csv";
            std::ofstream out(fs::path(out_dir) / name.str());
            out << "t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts,"
                   "pos_m,y_m,lat_err_m,grip_true_N,motor_force_N,motor_current_A,pc_beta\n";
            char line[512];

            int tick = 0;
            for (; tick < MAX_TICKS && !plant.finished(); ++tick) {
                const uint16_t adc = human.gripAdc();
                const int32_t counts = plant.encoderCounts();
                const MotorCommand cmd = engine.step(adc, counts);
                const HandInput hand = human.tick(tick);
                plant.advance(hand, cmd);

                if (tick % log_div == 0 || plant.finished()) {
                    const PlantState& s = plant.state();
                    std::snprintf(line, sizeof line,
                        "%d,%c,%u,%ld,%.5f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%.6f,%.7f,%.7f,%.4f,%.4f,%.4f,%.3f\n",
                        (tick + 1) * 2, mode, (unsigned)adc, (long)counts, engine.velocity(), engine.damping(),
                        engine.difficulty(), engine.tremorRms(), cmd.coast ? 0.0f : cmd.duty,
                        engine.observerEnergy() * 1000.0f, s.contacts, s.x, s.y, plant.lateralError(),
                        human.gripNewtons(), s.motor_force, s.motor_current, engine.passivityBeta());
                    out << line;
                }
            }
            TrialSummary ts{run, mode, plant.finished(), tick * (double)CONTROL_DT_SEC, plant.state().contacts,
                            plant.state().max_abs_current, engine.pcActivations(), engine.fault()};
            summaries.push_back(ts);
        }
    }

    // Manifest: parameters + per-trial summary (machine-readable provenance).
    std::ofstream man(fs::path(out_dir) / "manifest.json");
    man << "{\n  \"generator\": \"firmware/sim/test_bench.cpp\",\n  \"grip_model\": \"" << grip << "\",\n"
        << "  \"seed\": " << seed << ",\n  \"runs\": " << runs << ",\n  \"modes\": \"" << modes << "\",\n"
        << "  \"schmidt_k_s\": " << schmidt_k << ", \"pace_override\": " << pace_override << ",\n"
        << "  \"control_hz\": 500, \"plant_hz\": " << 500 * Plant::SUBSTEPS << ", \"log_hz\": " << 500 / log_div << ",\n"
        << "  \"counts_per_meter\": " << COUNTS_PER_METER << ",\n"
        << "  \"damping_fixed_mode_B\": " << b_fixed << ", \"damping_range_mode_C\": [" << DAMPING_MIN << ", " << DAMPING_MAX << "],\n"
        << "  \"plant\": {\"mass_x\": " << pp.mass_x << ", \"b_fric\": " << pp.b_fric << ", \"coulomb_n\": " << pp.coulomb_n
        << ", \"mass_y\": " << pp.mass_y << ", \"corridor_half_width_m\": " << pp.corridor_r << ", \"track_len_m\": " << pp.track_len
        << ", \"wire_amplitude_m\": " << pp.a_wire << ", \"wire_wavelength_m\": " << pp.lambda_wire
        << ", \"motor_kt_eff\": " << pp.motor.kt_eff << ", \"motor_r_ohm\": " << pp.motor.r_ohm << "},\n  \"trials\": [\n";
    for (size_t i = 0; i < summaries.size(); ++i) {
        const TrialSummary& t = summaries[i];
        man << "    {\"run\": " << t.run << ", \"mode\": \"" << t.mode << "\", \"completed\": " << (t.completed ? "true" : "false")
            << ", \"duration_s\": " << t.duration_s << ", \"contacts\": " << t.contacts
            << ", \"max_current_a\": " << t.max_current_a << ", \"pc_activations\": " << t.pc_activations
            << ", \"fault\": " << t.fault << "}" << (i + 1 < summaries.size() ? ",\n" : "\n");
    }
    man << "  ]\n}\n";

    int incomplete = 0;
    for (const auto& t : summaries) incomplete += t.completed ? 0 : 1;
    std::cout << "Wrote " << summaries.size() << " trials to " << out_dir << " (" << incomplete << " timed out)\n";
    return 0;
}
