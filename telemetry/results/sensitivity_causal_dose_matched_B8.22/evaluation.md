# Evaluation: `telemetry\data_sil\sensitivity\causal_dose_matched_B8.22`

- Trials analysed: 30 (30 completed the 180 mm path); excluded by validity rule: 0
- Generator: firmware/sim/test_bench.cpp, grip model: causal, Schmidt k: 0.04 s, seed: 2026
- Steering-law path length L = 182.58 mm, W = 2.70 mm, ID = L/W = 67.62 bit
- Passivity-controller active samples (all trials): 0
- Note: simulation runs are random seeds, not participants. p-values describe robustness to the simulated noise, not population-level inference.

| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |
|---|---|---|---|---|---|---|---|
| Contacts [count] | 0.4 ± 0.7 | 0.5 ± 0.97 | 0.4 ± 0.7 | 0.949 | +0 (1) | +0 (1) | +0 (1) |
| Movement time [s] | 3.61 ± 0.18 | 4.56 ± 0.32 | 4.55 ± 0.33 | 0.0005 | +1.03 (0.00586) | +1.01 (0.00586) | -0.021 (0.846) |
| Steering throughput (ID=L/W) [bit/s] | 18.8 ± 0.98 | 14.9 ± 1 | 14.9 ± 1.1 | 0.0005 | -4.02 (0.00586) | -3.99 (0.00586) | +0.0657 (0.77) |
| Lateral error SD [mm] | 0.439 ± 0.084 | 0.413 ± 0.11 | 0.412 ± 0.11 | 0.273 | -0.0569 (1) | -0.0483 (1) | -0.00319 (1) |
| Effective steering throughput (W_e) [bit/s] | 28.9 ± 5.8 | 25.4 ± 7.8 | 25.2 ± 6.7 | 0.045 | -1.75 (0.211) | -1.34 (0.111) | +0.138 (0.232) |
| Time in contact [%] | 0.376 ± 0.94 | 0.874 ± 1.6 | 0.831 ± 1.5 | 0.513 | +0 (0.875) | +0 (0.875) | +0 (0.75) |
| Longitudinal RMS jerk [m/s^3] | 9.5 ± 2.2 | 8.04 ± 1.8 | 8.03 ± 1.8 | 0.0005 | -1.6 (0.00586) | -1.57 (0.00586) | +0.0113 (0.846) |
| Transverse RMS jerk [m/s^3] | 15 ± 2.7 | 13.7 ± 2.1 | 13.7 ± 2 | 0.0608 | -1.24 (0.0293) | -1.2 (0.0293) | -0.0514 (0.625) |
| Mean applied damping [N*s/m] | 0 ± 0 | 8.22 ± 1.8e-15 | 8.22 ± 0.77 | 0.000553 | +8.22 (0.00586) | +8.22 (0.00586) | +0.000409 (1) |

Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.

## Verdicts

- Contacts: no significant B vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs B difference (median diff +0 count, Holm p = 1).
- Movement time: mode B is higher than mode A (worse): median diff +1.03 s, 95% CI [+0.696, +1.13], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is higher than mode A (worse): median diff +1.01 s, 95% CI [+0.656, +1.23], r_rb = +1.00, Holm p = 0.00586.
- Movement time: no significant C vs B difference (median diff -0.021 s, Holm p = 0.846).
- Steering throughput (ID=L/W): mode B is lower than mode A (worse): median diff -4.02 bit/s, 95% CI [-4.91, -3.09], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is lower than mode A (worse): median diff -3.99 bit/s, 95% CI [-4.79, -2.94], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): no significant C vs B difference (median diff +0.0657 bit/s, Holm p = 0.77).
- Lateral error SD: no significant B vs A difference (median diff -0.0569 mm, Holm p = 1).
- Lateral error SD: no significant C vs A difference (median diff -0.0483 mm, Holm p = 1).
- Lateral error SD: no significant C vs B difference (median diff -0.00319 mm, Holm p = 1).
- Effective steering throughput (W_e): no significant B vs A difference (median diff -1.75 bit/s, Holm p = 0.211).
- Effective steering throughput (W_e): no significant C vs A difference (median diff -1.34 bit/s, Holm p = 0.111).
- Effective steering throughput (W_e): no significant C vs B difference (median diff +0.138 bit/s, Holm p = 0.232).
- Time in contact: no significant B vs A difference (median diff +0 %, Holm p = 0.875).
- Time in contact: no significant C vs A difference (median diff +0 %, Holm p = 0.875).
- Time in contact: no significant C vs B difference (median diff +0 %, Holm p = 0.75).
- Longitudinal RMS jerk: mode B is lower than mode A (better): median diff -1.6 m/s^3, 95% CI [-2.03, -1.04], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: mode C is lower than mode A (better): median diff -1.57 m/s^3, 95% CI [-2.03, -1.07], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: no significant C vs B difference (median diff +0.0113 m/s^3, Holm p = 0.846).
- Transverse RMS jerk: mode B is lower than mode A (better): median diff -1.24 m/s^3, 95% CI [-1.75, -0.446], r_rb = -0.85, Holm p = 0.0293.
- Transverse RMS jerk: mode C is lower than mode A (better): median diff -1.2 m/s^3, 95% CI [-1.74, -0.464], r_rb = -0.89, Holm p = 0.0293.
- Transverse RMS jerk: no significant C vs B difference (median diff -0.0514 m/s^3, Holm p = 0.625).
- Mean applied damping: mode B is higher than mode A: median diff +8.22 N*s/m, 95% CI [+8.22, +8.22], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is higher than mode A: median diff +8.22 N*s/m, 95% CI [+7.55, +8.73], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: no significant C vs B difference (median diff +0.000409 N*s/m, Holm p = 1).
