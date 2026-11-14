# Evaluation: `telemetry\data_sil\causal`

- Trials analysed: 30 (30 completed the 180 mm path); excluded by validity rule: 0
- Generator: firmware/sim/test_bench.cpp, grip model: causal, Schmidt k: 0.04 s, seed: 2026
- Steering-law path length L = 182.58 mm, W = 2.70 mm, ID = L/W = 67.62 bit
- Passivity-controller active samples (all trials): 0
- Note: simulation runs are random seeds, not participants. p-values describe robustness to the simulated noise, not population-level inference.

| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |
|---|---|---|---|---|---|---|---|
| Contacts [count] | 0.4 ± 0.7 | 0 ± 0 | 0.4 ± 0.7 | 0.223 | +0 (0.75) | +0 (1) | +0 (0.75) |
| Movement time [s] | 3.61 ± 0.18 | 5.04 ± 0.33 | 4.55 ± 0.33 | 4.54e-05 | +1.5 (0.00586) | +1.01 (0.00586) | -0.538 (0.00586) |
| Steering throughput (ID=L/W) [bit/s] | 18.8 ± 0.98 | 13.5 ± 0.88 | 14.9 ± 1.1 | 4.54e-05 | -5.54 (0.00586) | -3.99 (0.00586) | +1.61 (0.00586) |
| Lateral error SD [mm] | 0.439 ± 0.084 | 0.361 ± 0.067 | 0.412 ± 0.11 | 0.00745 | -0.0902 (0.00586) | -0.0483 (0.375) | +0.0418 (0.129) |
| Effective steering throughput (W_e) [bit/s] | 28.9 ± 5.8 | 25.2 ± 5.4 | 25.2 ± 6.7 | 0.0672 | -3.32 (0.111) | -1.34 (0.111) | -0.159 (1) |
| Time in contact [%] | 0.376 ± 0.94 | 0 ± 0 | 0.831 ± 1.5 | 0.247 | +0 (0.875) | +0 (0.875) | +0 (0.75) |
| Longitudinal RMS jerk [m/s^3] | 9.5 ± 2.2 | 7.91 ± 1.8 | 8.03 ± 1.8 | 0.000371 | -1.66 (0.00586) | -1.57 (0.00586) | +0.155 (0.193) |
| Transverse RMS jerk [m/s^3] | 15 ± 2.7 | 13.7 ± 2.6 | 13.7 ± 2 | 0.0608 | -1.09 (0.0293) | -1.2 (0.0293) | +0.126 (0.77) |
| Mean applied damping [N*s/m] | 0 ± 0 | 13 ± 0 | 8.22 ± 0.77 | 4.54e-05 | +13 (0.00586) | +8.22 (0.00586) | -4.78 (0.00586) |

Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.

## Verdicts

- Contacts: no significant B vs A difference (median diff +0 count, Holm p = 0.75).
- Contacts: no significant C vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs B difference (median diff +0 count, Holm p = 0.75).
- Movement time: mode B is higher than mode A (worse): median diff +1.5 s, 95% CI [+1.22, +1.64], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is higher than mode A (worse): median diff +1.01 s, 95% CI [+0.656, +1.23], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is lower than mode B (better): median diff -0.538 s, 95% CI [-0.632, -0.308], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode B is lower than mode A (worse): median diff -5.54 bit/s, 95% CI [-5.84, -4.83], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is lower than mode A (worse): median diff -3.99 bit/s, 95% CI [-4.79, -2.94], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is higher than mode B (better): median diff +1.61 bit/s, 95% CI [+0.964, +1.96], r_rb = +1.00, Holm p = 0.00586.
- Lateral error SD: mode B is lower than mode A (better): median diff -0.0902 mm, 95% CI [-0.108, -0.0149], r_rb = -1.00, Holm p = 0.00586.
- Lateral error SD: no significant C vs A difference (median diff -0.0483 mm, Holm p = 0.375).
- Lateral error SD: no significant C vs B difference (median diff +0.0418 mm, Holm p = 0.129).
- Effective steering throughput (W_e): no significant B vs A difference (median diff -3.32 bit/s, Holm p = 0.111).
- Effective steering throughput (W_e): no significant C vs A difference (median diff -1.34 bit/s, Holm p = 0.111).
- Effective steering throughput (W_e): no significant C vs B difference (median diff -0.159 bit/s, Holm p = 1).
- Time in contact: no significant B vs A difference (median diff +0 %, Holm p = 0.875).
- Time in contact: no significant C vs A difference (median diff +0 %, Holm p = 0.875).
- Time in contact: no significant C vs B difference (median diff +0 %, Holm p = 0.75).
- Longitudinal RMS jerk: mode B is lower than mode A (better): median diff -1.66 m/s^3, 95% CI [-1.87, -1.13], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: mode C is lower than mode A (better): median diff -1.57 m/s^3, 95% CI [-2.03, -1.07], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: no significant C vs B difference (median diff +0.155 m/s^3, Holm p = 0.193).
- Transverse RMS jerk: mode B is lower than mode A (better): median diff -1.09 m/s^3, 95% CI [-2.18, -0.185], r_rb = -0.85, Holm p = 0.0293.
- Transverse RMS jerk: mode C is lower than mode A (better): median diff -1.2 m/s^3, 95% CI [-1.74, -0.464], r_rb = -0.89, Holm p = 0.0293.
- Transverse RMS jerk: no significant C vs B difference (median diff +0.126 m/s^3, Holm p = 0.77).
- Mean applied damping: mode B is higher than mode A: median diff +13 N*s/m, 95% CI [+13, +13], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is higher than mode A: median diff +8.22 N*s/m, 95% CI [+7.55, +8.73], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is lower than mode B: median diff -4.78 N*s/m, 95% CI [-5.45, -4.27], r_rb = -1.00, Holm p = 0.00586.
