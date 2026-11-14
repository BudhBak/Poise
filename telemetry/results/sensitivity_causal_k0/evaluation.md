# Evaluation: `telemetry\data_sil\sensitivity\causal_k0`

- Trials analysed: 30 (30 completed the 180 mm path); excluded by validity rule: 0
- Generator: firmware/sim/test_bench.cpp, grip model: causal, Schmidt k: 0 s, seed: 2026
- Steering-law path length L = 182.58 mm, W = 2.70 mm, ID = L/W = 67.62 bit
- Passivity-controller active samples (all trials): 0
- Note: simulation runs are random seeds, not participants. p-values describe robustness to the simulated noise, not population-level inference.

| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |
|---|---|---|---|---|---|---|---|
| Contacts [count] | 0 ± 0 | 0 ± 0 | 0 ± 0 | 1 | +0 (1) | +0 (1) | +0 (1) |
| Movement time [s] | 3.5 ± 0.23 | 5.04 ± 0.33 | 4.45 ± 0.3 | 4.54e-05 | +1.53 (0.00586) | +0.927 (0.00586) | -0.556 (0.00586) |
| Steering throughput (ID=L/W) [bit/s] | 19.4 ± 1.3 | 13.5 ± 0.88 | 15.3 ± 1 | 4.54e-05 | -5.88 (0.00586) | -4.13 (0.00586) | +1.83 (0.00586) |
| Lateral error SD [mm] | 0.234 ± 0.047 | 0.204 ± 0.045 | 0.204 ± 0.041 | 0.000553 | -0.0225 (0.00586) | -0.0295 (0.00586) | +0.00298 (1) |
| Effective steering throughput (W_e) [bit/s] | 56.2 ± 11 | 45.3 ± 11 | 50.7 ± 12 | 0.00166 | -12.1 (0.0117) | -4.84 (0.0137) | +4.42 (0.0117) |
| Time in contact [%] | 0 ± 0 | 0 ± 0 | 0 ± 0 | 1 | +0 (1) | +0 (1) | +0 (1) |
| Longitudinal RMS jerk [m/s^3] | 9.36 ± 2.3 | 7.91 ± 1.8 | 7.91 ± 1.8 | 0.000553 | -1.4 (0.00586) | -1.31 (0.00586) | +0.00305 (0.846) |
| Transverse RMS jerk [m/s^3] | 13.1 ± 3.1 | 12.7 ± 2.9 | 12.6 ± 2.6 | 0.15 | -0.43 (0.32) | -0.243 (0.193) | -0.0222 (0.77) |
| Mean applied damping [N*s/m] | 0 ± 0 | 13 ± 0 | 8.03 ± 0.6 | 4.54e-05 | +13 (0.00586) | +8.06 (0.00586) | -4.94 (0.00586) |

Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.

## Verdicts

- Contacts: no significant B vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs B difference (median diff +0 count, Holm p = 1).
- Movement time: mode B is higher than mode A (worse): median diff +1.53 s, 95% CI [+1.46, +1.64], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is higher than mode A (worse): median diff +0.927 s, 95% CI [+0.89, +1.02], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is lower than mode B (better): median diff -0.556 s, 95% CI [-0.696, -0.528], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode B is lower than mode A (worse): median diff -5.88 bit/s, 95% CI [-6.19, -5.55], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is lower than mode A (worse): median diff -4.13 bit/s, 95% CI [-4.41, -3.79], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is higher than mode B (better): median diff +1.83 bit/s, 95% CI [+1.57, +1.96], r_rb = +1.00, Holm p = 0.00586.
- Lateral error SD: mode B is lower than mode A (better): median diff -0.0225 mm, 95% CI [-0.0337, -0.0158], r_rb = -1.00, Holm p = 0.00586.
- Lateral error SD: mode C is lower than mode A (better): median diff -0.0295 mm, 95% CI [-0.0452, -0.0063], r_rb = -1.00, Holm p = 0.00586.
- Lateral error SD: no significant C vs B difference (median diff +0.00298 mm, Holm p = 1).
- Effective steering throughput (W_e): mode B is lower than mode A (worse): median diff -12.1 bit/s, 95% CI [-16.2, -9.03], r_rb = -0.96, Holm p = 0.0117.
- Effective steering throughput (W_e): mode C is lower than mode A (worse): median diff -4.84 bit/s, 95% CI [-9.88, -1.69], r_rb = -0.85, Holm p = 0.0137.
- Effective steering throughput (W_e): mode C is higher than mode B (better): median diff +4.42 bit/s, 95% CI [+3.22, +7.65], r_rb = +0.93, Holm p = 0.0117.
- Time in contact: no significant B vs A difference (median diff +0 %, Holm p = 1).
- Time in contact: no significant C vs A difference (median diff +0 %, Holm p = 1).
- Time in contact: no significant C vs B difference (median diff +0 %, Holm p = 1).
- Longitudinal RMS jerk: mode B is lower than mode A (better): median diff -1.4 m/s^3, 95% CI [-1.66, -1.11], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: mode C is lower than mode A (better): median diff -1.31 m/s^3, 95% CI [-1.78, -1.07], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: no significant C vs B difference (median diff +0.00305 m/s^3, Holm p = 0.846).
- Transverse RMS jerk: no significant B vs A difference (median diff -0.43 m/s^3, Holm p = 0.32).
- Transverse RMS jerk: no significant C vs A difference (median diff -0.243 m/s^3, Holm p = 0.193).
- Transverse RMS jerk: no significant C vs B difference (median diff -0.0222 m/s^3, Holm p = 0.77).
- Mean applied damping: mode B is higher than mode A: median diff +13 N*s/m, 95% CI [+13, +13], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is higher than mode A: median diff +8.06 N*s/m, 95% CI [+7.44, +8.72], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is lower than mode B: median diff -4.94 N*s/m, 95% CI [-5.56, -4.28], r_rb = -1.00, Holm p = 0.00586.
