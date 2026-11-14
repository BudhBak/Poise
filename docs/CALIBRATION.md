# Simulation protocol and operator-model calibration

All between-mode results in this repository depend on `firmware/sim/operator_model.h`. This document records how that model was built and tuned, so a reader can judge how much the results owe to the author's choices.

## Rules followed

1. **No oracle.** The controller (`admittance.h`) sees only the grip ADC value and encoder counts. The operator model sees only causal, delayed and noisy information: vision delayed 80–120 ms through a 5–15 mm preview band, proprioception delayed 30 ms, contact sound delayed 60 ms. Neither ever reads the true lateral error or the wall penetration.
2. **Calibrate on Mode A only.** Every parameter was tuned against Mode A (no assistance) behaviour before any Mode B or C run was looked at. The parameters were then frozen.
3. **Pre-registered analysis.** The primary dataset is seed 2026, 10 seeds, Schmidt k = 0.04 s. Mode B damping is 13.0 N·s/m, the midpoint of the Mode C range. The null-grip model and the k ∈ {0, 0.08} s runs are sensitivity analyses. The dose-matched run (B = 8.22 N·s/m) was added **after** the primary results were seen. It is labelled as a post-hoc control, and its purpose is to test whether C's advantage comes from adaptation or just from a lower average dose.
4. **Common random numbers.** For each seed, modes A, B and C share the operator parameters and the noise stream, so they form paired comparisons.

## Calibration history (Mode A only)

| Step | Problem observed in Mode A | Change |
|---|---|---|
| 1 | Grip saturated at the top of its range, and the tremor band rang | The preview window was too long, so curvature summed over too much path. It became a 5–15 mm preview band using mean curvature. Intrinsic axial muscle viscosity (b_intrinsic_x = 12 N·s/m) was added below the delayed velocity loop (k_v = 15). |
| 2 | No speed–accuracy trade-off: faster runs were not less accurate | Added Schmidt-law lateral command noise, σ_y = k·\|dy_cmd/dt\|. A bug was found and fixed: the aim rate had been derived from noisy perceptions, and it now uses the intended lateral speed (slope × perceived axial speed). |
| 3 | Contacts still *fell* as speed rose | Lateral tremor reduced to a normal physiological level (0.015 N), and co-contraction stiffness (gain 1.0 per unit grip) added. |

Final Mode A sweep over pace: lateral SD rises with speed for every k. Contacts by pace: at k = 0.04 s they go from 0.1 to 3.8; at k = 0.08 s from 3.5 to 9.5 (Spearman ρ = 0.41, p = 0.003). The model therefore shows the expected speed–accuracy trade-off before any assistance is applied.

## Grip models

- **Causal (primary):** grip = 2 N rest + 3.5 N × (previewed path demand) + 2.5 N × (startle after a perceived contact) + Ornstein–Uhlenbeck stress (σ 0.6 N, τ 2 s). Grip rises with what the simulated person sees coming, never with their actual error.
- **Null (control):** grip = 2 N + OU stress (σ 1.4 N, τ 1 s). It carries no task information.

## Sensitivity summary (C − B, paired median, Holm p)

| Dataset | Movement time | Lateral SD |
|---|---|---|
| Causal, k = 0.04 (primary) | −0.538 s (0.0059) | +0.042 mm (0.13) |
| Null grip | −0.897 s (0.0059) | +0.083 mm (0.0059) |
| Causal, k = 0 | −0.556 s (0.0059) | +0.003 mm (1) |
| Causal, k = 0.08 | −0.086 s (0.38) | +0.046 mm (0.064) |
| Dose-matched B = 8.22 (post hoc) | −0.021 s (0.85) | −0.003 mm (1) |

Reading: C's speed advantage over B = 13 is a dose effect, not an adaptation effect. When the doses are matched it disappears, and it appears just as strongly when grip carries no task information.

## Known modelling limits

- Grip-to-demand coupling (3.5 N per unit demand) is a design assumption, not a measurement. If real people clench more (or less) on hard sections, adaptation could help more (or less).
- Tremor is modelled as an 8–11 Hz sinusoid plus noise. It does not include pathological tremor, fatigue, or learning across trials.
- No inter-subject variability beyond the per-seed draws of pace (±8 %), tremor amplitude (±20 %), tremor frequency and visual delay.

## Desktop bench note

In `hil_trackpad_bench.py --selftest` the scripted pointer is open-loop: its y follows the wire at the pointer's own x, not at the stylus's x. With damping on, the stylus lags in x, so the scripted run collects contacts in B and C (47 and 38 in the logged self-test) but not in A. That is an artefact of a non-visual script, not a performance measure. A human sees the stylus and corrects for the lag.
