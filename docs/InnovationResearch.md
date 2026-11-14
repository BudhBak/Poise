# v3 innovation research: candidate controllers, analysis and pre-registration

This document defines the v3 candidate controllers and their analysis, and fixes the confirmatory protocol **before** any endpoint suite (S1–S5) was run. Its SHA-256 was written to `telemetry/results/innovation/prereg.sha256` before the first confirmatory run. `analyze_innovations.py` recomputes the hash and reports whether this file has changed since then.

The reference controller is v2 Mode C (`firmware/src/admittance.h`), unchanged. The candidate engine is `firmware/sim/innovation_engine.h`, the lab is `firmware/sim/innovation_lab.cpp`, and the analysis is `telemetry/scripts/analyze_innovations.py`. Everything here is simulation. No candidate has run on hardware or with a person.

## 1. Contract kept by every candidate

- **Inputs:** grip ADC counts and encoder counts at 500 Hz (`InnovationEngine::step`, `innovation_engine.h:112`). There is no geometry, wall, trial-phase or clock input. The only exception is the breathing-cue swell, which uses its own tick counter, as the v2 LED pacer does.
- **Shared with v2:** exact ZOH passivity observer, 25 Hz velocity LPF, FSR inverse model with 20 ms EMA, raw-velocity passivity controller with the back-EMF feed-forward dropped while β > 0, current limit, deadband, and overspeed latch (`innovation_engine.h:124-135`, `:213-239`).
- **Parity check:** suite PARITY in `innovation_lab.cpp:433`. With fixed B = 13, the engine reproduces v2 Mode B bit-for-bit, and the lab's B13 trials equal the committed SIL data for every seed (recorded in the previous session).
- **Implementation limits:** float only, no heap, every loop bounded at 16 iterations.

## 2. Structural bound (diagnostic D0, Mode A, seed 2026, 10 seeds)

Before any candidate was tuned, D0 removed the operator's x-axis tremor entirely. This is an oracle no controller can beat.

| Metric | Nominal | x-tremor removed |
|---|---|---|
| Lateral SD | 0.4389 mm | 0.4383 mm (−1.8 %, p = 1) |
| Contacts | 0.4 | 0.3 |
| Movement time | 3.61 s | 3.58 s |
| Longitudinal jerk | 9.50 m/s³ | 0.99 m/s³ (−90 %) |
| 6–14 Hz x displacement | 89 µm | 6.4 µm |

The motor acts on x only. Lateral error comes from Schmidt-law y-command noise (`operator_model.h:152-158`). So even a perfect x-tremor canceller moves lateral SD by under 2 %. **In this model no x-axis controller can meaningfully break the speed–accuracy frontier.** The speed–smoothness frontier (longitudinal jerk at equal movement time) is the one an x-axis controller can move. The promotion rule (section 8) still requires an accuracy-frontier break, as the task specification demands, so a promotion is not expected.

## 3. Candidates

| Name | Force law (see `innovation_engine.h:195-210`) | Adaptive elements |
|---|---|---|
| DB_BMFLC | F = −B_vol·v_f − (B_tr − B_vol)·v_tr, B_vol = 3.75, B_tr = 33.5 N·s/m | none (fixed dual band) |
| DB_LTI | same, v_tr from one RBJ band-pass (9 Hz, Q 1.2) | none |
| DB_ADAPT | dual band, B_vol 2.5–5.0, B_tr 22–45 | d from tremor × (1 + ρ⁺·grip), F0 baseline, surge anticipator |
| FO_SUB | F = −B_vol·N(α)·D^(α−1) v_raw, α 0.55–0.95 | α from d |
| FO_SUPER | same, α 1.05–1.45 | α from d |
| DB_VI | DB_BMFLC + tank-gated virtual inertia −M·a_tr, M ≤ 1 kg | M gated by observer energy |
| FULL_NODS | DB_ADAPT + virtual inertia | all of the above |
| FULL | FULL_NODS + 2.5 % duty dither + 20 % 0.1 Hz damping swell | all |
| SCALAR_F0 | v2 scalar law (B 4.5–21.5) with F0 tracking, ρ gate, anticipator | d with new grip pathway |

The B_vol / B_tr values were chosen during development. No confirmatory data were seen, and no optimality derivation is claimed. The difficulty law and B_vol slew/smoother are as in v2. The tremor-band damping follows d through a 20 ms EMA.

## 4. Tremor estimator analysis (BMFLC)

The estimator is a band-limited multiple Fourier linear combiner: 16 quadrature bins from 6.0 to 13.5 Hz at 0.5 Hz spacing. It runs on raw velocity after a 4 Hz second-order Butterworth high-pass (`innovation_engine.h:303-331`).

- **The textbook step size is broadband.** With μ = 0.02 the aggregate loop gain is 2μN = 0.64, so the "band-limited" estimator passes nearly everything. Following Glover's LTI analysis of LMS with fixed sinusoidal references, each bin's transfer function on the unit circle is G_k = −μ + jμ·sinθ/(cosθ − cos θ_k). The bins interfere: between adjacent bins there are zeros with phase inversion (example: 9.25 Hz, gain 0.145 at +177°).
- **Closed-loop poles are lightly damped.** The state-space poles have |p| = 0.99944 (τ ≈ 3.6 s, or 29 s at μ = 0.02). This produced 2 s realignment spikes in development runs. A leak of 0.002–0.005 suppresses them but cuts in-band gain to 0.2–0.35. An incommensurate (golden-ratio-jittered) grid was tested and not adopted (`BM_JITTER = 0`).
- **Adopted:** μ = π·Δf·T = 0.00314 (per-bin bandwidth ≈ bin spacing) and leak 0.001 (τ = 2 s). Measured bin gain is ≈ 1.0 with ≈ 3° phase error, and 2 Hz leaks through at 0.15.
- **Phase compensation:** each bin's complex amplitude is multiplied by K_k = e^{jθ_k/2} / (H_bd·H_hp). This removes the high-pass and backward-difference responses and advances half a sample (`innovation_engine.h:277-297`).
- **Envelope:** the magnitude of the analytic tremor signal, √(½(v_t² + v_q²)). The sum of squared bin weights is not used. The bins are not orthogonal at this spacing and memory, so the weights can grow in cancelling directions and overstate power.

## 5. Calibration (development seed 1001 only, before any confirmatory run)

- **Quantisation-only glides** at 0.01–0.20 m/s (suite CAL): maximum envelope 0.00852 m/s (`telemetry/data_lab/cal_dev1001/quant.json`).
- **Mode A envelope**, sensing-only BMFLC, 10 seeds: median 0.0022, p90 0.0042, max 0.0155 m/s.
- **Setting:** TR_ENV_FLOOR = TR_ENV_SPAN = 0.0086 m/s (`innovation_engine.h:69-70`). Tremor-driven adaptation therefore rarely engages at physiological tremor amplitude. This consequence was known before the confirmatory runs. Suite S2b (tremor ×4) is where tremor adaptation can act.

## 6. Fractional-order damper

The operator is a 16-tap Grünwald–Letnikov approximation of D^(α−1) on raw velocity, normalised to unit gain at 2 Hz (`innovation_engine.h:341-353`).

- **α < 1** gives less high-frequency damping than a viscous damper. The truncated GL sum also saturates below ~5 Hz, since 16 taps at 2 ms is 32 ms of memory. FO_SUB therefore adds *less* damping in the tremor band. Prediction P3 follows from this.
- **α > 1** raises the gain toward Nyquist. That is the failure mode the sampled-data passivity condition penalises (section 7, ADV result).

## 7. Passivity, quantisation and timing method

**Frequency domain.** The generalised Colgate–Schenkel condition for the implemented chain is

  b_phys > max_θ −Re{e^{−jθ}·H(e^{jθ})},  where b_phys = b_friction + K_t²/R = 1.0 + 5.6²/2.2 = 15.25 N·s/m.

- On the feed-forward path, H = G − c·b_emf·L, with c = 1 and L = the 25 Hz LPF.
- On the passivity-controller fallback path (feed-forward dropped), H = G + β_max, with β_max = 10.
- G is the *measured* rendered impedance of the frozen-difficulty force law at d = 0 and d = 1. It is measured on 320 log-spaced frequencies from 0.2 to 250 Hz, including Nyquist (`innovation_lab.cpp:339`).
- For G = B·L this reduces exactly to `verify_control_theory.py` V5. Check: B13 margin 14.00 and C_v2 margin 5.50 N·s/m, the latter matching V5.

**Time domain** (same criteria as V6/V7, plus reversals):
- Nominal 20 s closed loop: work ≤ 0.
- 2.5× back-EMF over-compensation with the PC on: work ≤ 5 mJ. It must exceed that with the PC off, otherwise the test is meaningless.
- Prescribed 8 Hz, 4 mm reversals: nominal work ≤ 0, and ≤ 5 mJ under over-compensation.
- Quantisation-only glides: d > 0.05 in ≤ 1 % of ticks.
- Superposition error below 10⁻³ at frozen d.

**Timing.** `telemetry/scripts/arm_cycle_budget.py` compiles `firmware/sim/arm_probe.cpp` with the Arduino renesas_uno flags (−Os, cortex-m4, fpv4-sp-d16 hard float). It disassembles the result and bounds every path:
- Cortex-M4 TRM upper bounds per instruction, branches always taken.
- Each CFG loop charged ×16.
- Library calls charged sqrtf 40, fmodf 250 and cosf 400 cycles.
- ×2 for flash wait states.

Results (run before this document was hashed):

| Probe | cycles (0 wait) | ×2 bound | µs of 250 µs budget |
|---|---|---|---|
| probe_v2 (v2 HapticEngine) | 807 | 1614 | 33.6 |
| probe_db (dual-band BMFLC) | 4941 | 9882 | 205.9 |
| probe_worst (every v3 feature) | 4941 | 9882 | 205.9 |

Compiler warnings 0; no double-precision helper calls. `stepCore` has 4 detected loops in 642 instructions. Because every branch is summed, probe_db and probe_worst have the same bound. The bound fits the budget, but with only 18 % headroom.

**ADV outcome**, run before hashing: verification only, no task endpoint was computed. Margins in N·s/m.

| Condition | C-S margin | fallback margin | over-comp PC on (mJ) | verdict |
|---|---|---|---|---|
| B13 (v2) | +14.00 | +5.25 | −1.39 | pass |
| C_v2 | +5.50 | +5.25 | +4.98 | pass |
| DB_BMFLC | +0.66 | +3.94 | +1.36 | pass |
| DB_LTI | +4.75 | +5.25 | **+8.34** | **fail** (over-comp bound) |
| DB_ADAPT | +0.42 | +3.49 | −8.05 | pass |
| FO_SUB | +4.76 | +2.18 | −6.47 | pass |
| FO_SUPER | **−11.23** | **−21.23** | −9.28 | **fail** (HF gain) |
| DB_VI | **−27.51** | **−4.02** | +1.36 | **fail** (Re G = −29.9 at 4.9 Hz) |
| FULL_NODS / FULL | **−31.77** | **−8.35** | −8.05 / −8.69 | **fail** (virtual inertia) |
| SCALAR_F0 | +5.50 | +5.25 | +3.91 | pass |

The candidates were **not** modified after these results. Failing candidates still run in every suite for information, but rule (c) makes them ineligible for promotion. Virtual inertia rendered from a band-limited acceleration estimate is passive only through the energy tank (time-domain closed loops pass). It does not satisfy the frequency-domain condition at full mass.

## 8. Confirmatory protocol and promotion rule

**Data.** Seed 2026, N = 10 matched seeds (common random numbers, identical operator draws to `test_bench.cpp`). The suites are:

| Suite | Condition |
|---|---|
| S1 | primary: causal operator, k = 0.04 s |
| S2a | k = 0.08 s |
| S2b | tremor force ×4 |
| S3 | FSR reads a steady 8 N, operator tremor scale 0.5 |
| S4 | resting grip 1.5 / 3.0 / 4.5 / 6.0 N (sensor offset) |
| S5 | DB_ADAPT vs +0.1 Hz swell vs +dither |

**Endpoints.**
- Primary: movement time, lateral SD, contacts.
- Secondary: Accot–Zhai throughput (ID/MT and W_e), longitudinal jerk, transverse jerk, and 6–14 Hz x displacement. The displacement is the RMS of the 4th-order Butterworth band-pass of logged position, zero-phase, with 0.5 s edges trimmed.

**Tests.** Paired two-sided Wilcoxon signed-rank tests, Holm-corrected within each family:
- Primary family: 3 metrics × candidates × {C_v2, B8.22}.
- Secondary family: 5 metrics × candidates × {C_v2, B8.22}.
- Frontier family: {lateral SD, contacts, longitudinal jerk} × candidates. The residual is computed per seed as the candidate's value minus the fixed-damper curve (A, B4.5, B8.22, B13, B20), linearly interpolated at the candidate's own MT.

**Promotion into `firmware/src/` requires all three, in S1:**
- (a) a negative lateral-SD or contacts frontier residual with Holm p < 0.05;
- (b) against **both** C_v2 and B8.22, significantly better on ≥ 1 primary endpoint and significantly worse on none;
- (c) every section 7 check passes, and the cycle bound fits.

Robustness suites S2–S5 are reported but do not gate promotion.

## 9. Predictions (fixed before S1–S5)

- **P1.** No candidate breaks the speed–accuracy frontier: |lateral-SD frontier residual| ≤ ~2 % and not significant.
- **P2.** Dual-band candidates (DB_BMFLC, DB_LTI, DB_ADAPT) reduce longitudinal jerk below the fixed-damper curve (negative jerk frontier residual).
- **P3.** FO_SUB is worse than FO_SUPER on longitudinal jerk.
- **P4.** Dither (2.5 % duty) increases longitudinal jerk vs. DB_ADAPT (S5).
- **P5.** Relative to B8.22, v2 Mode C is slower than it is in S1 under a firm 8 N grip (S3) and at a 6 N resting grip (S4).

Expected consequence: no promotion. P1 combined with rule (a) makes a promotion unlikely by construction of the operator model.

**Not testable: suite 5's breathing recovery** (visual vs visuo-haptic pacer). The operator model has no respiration or arousal state, so recovery cannot be measured. Only non-interference (S5) is run. Claiming a recovery benefit would require a human study.

## 10. Hash record

`telemetry/results/innovation/prereg.sha256` holds the SHA-256 of this file, written before suite S1 was first run. The analysis prints the current hash and whether it matches.
