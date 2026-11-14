# Evaluation: `telemetry\data_sil\sensitivity\causal_k0.08`

- Trials analysed: 30 (30 completed the 180 mm path); excluded by validity rule: 0
- Generator: firmware/sim/test_bench.cpp, grip model: causal, Schmidt k: 0.08 s, seed: 2026
- Steering-law path length L = 182.58 mm, W = 2.70 mm, ID = L/W = 67.62 bit
- Passivity-controller active samples (all trials): 0
- Note: simulation runs are random seeds, not participants. p-values describe robustness to the simulated noise, not population-level inference.

| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |
|---|---|---|---|---|---|---|---|
| Contacts [count] | 8.3 ± 4.1 | 5.4 ± 2.8 | 7 ± 3.5 | 0.146 | -2.5 (0.0586) | -0.5 (0.59) | +2.5 (0.273) |
| Movement time [s] | 4.48 ± 0.41 | 5.85 ± 0.47 | 5.6 ± 0.57 | 0.0005 | +1.45 (0.00586) | +0.851 (0.00586) | -0.086 (0.375) |
| Steering throughput (ID=L/W) [bit/s] | 15.2 ± 1.4 | 11.6 ± 0.92 | 12.2 ± 1.2 | 0.0005 | -3.59 (0.00586) | -2.48 (0.00586) | +0.206 (0.322) |
| Lateral error SD [mm] | 0.767 ± 0.14 | 0.598 ± 0.096 | 0.648 ± 0.096 | 0.00335 | -0.187 (0.00586) | -0.088 (0.0273) | +0.0462 (0.0645) |
| Effective steering throughput (W_e) [bit/s] | 13.4 ± 3 | 13.2 ± 3.6 | 12.6 ± 2.6 | 0.905 | -0.104 (1) | -0.381 (1) | -0.302 (1) |
| Time in contact [%] | 11.8 ± 9.6 | 4.04 ± 2.4 | 7.26 ± 3.4 | 0.045 | -6.71 (0.041) | -4.64 (0.16) | +3.58 (0.041) |
| Longitudinal RMS jerk [m/s^3] | 11 ± 2.3 | 8.95 ± 1.8 | 9.17 ± 1.8 | 0.000371 | -1.88 (0.00586) | -1.71 (0.00586) | +0.271 (0.322) |
| Transverse RMS jerk [m/s^3] | 18.6 ± 2.5 | 16.6 ± 2.8 | 16.9 ± 1.7 | 0.15 | -2.4 (0.082) | -1.53 (0.082) | +0.00371 (0.77) |
| Mean applied damping [N*s/m] | 0 ± 0 | 13 ± 0 | 10.4 ± 0.97 | 4.54e-05 | +13 (0.00586) | +10.2 (0.00586) | -2.8 (0.00586) |

Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.

## Verdicts

- Contacts: no significant B vs A difference (median diff -2.5 count, Holm p = 0.0586).
- Contacts: no significant C vs A difference (median diff -0.5 count, Holm p = 0.59).
- Contacts: no significant C vs B difference (median diff +2.5 count, Holm p = 0.273).
- Movement time: mode B is higher than mode A (worse): median diff +1.45 s, 95% CI [+1.09, +1.61], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is higher than mode A (worse): median diff +0.851 s, 95% CI [+0.6, +1.74], r_rb = +1.00, Holm p = 0.00586.
- Movement time: no significant C vs B difference (median diff -0.086 s, Holm p = 0.375).
- Steering throughput (ID=L/W): mode B is lower than mode A (worse): median diff -3.59 bit/s, 95% CI [-4.42, -2.7], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is lower than mode A (worse): median diff -2.48 bit/s, 95% CI [-4.58, -1.69], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): no significant C vs B difference (median diff +0.206 bit/s, Holm p = 0.322).
- Lateral error SD: mode B is lower than mode A (better): median diff -0.187 mm, 95% CI [-0.252, -0.0745], r_rb = -1.00, Holm p = 0.00586.
- Lateral error SD: mode C is lower than mode A (better): median diff -0.088 mm, 95% CI [-0.235, -0.0136], r_rb = -0.85, Holm p = 0.0273.
- Lateral error SD: no significant C vs B difference (median diff +0.0462 mm, Holm p = 0.0645).
- Effective steering throughput (W_e): no significant B vs A difference (median diff -0.104 bit/s, Holm p = 1).
- Effective steering throughput (W_e): no significant C vs A difference (median diff -0.381 bit/s, Holm p = 1).
- Effective steering throughput (W_e): no significant C vs B difference (median diff -0.302 bit/s, Holm p = 1).
- Time in contact: mode B is lower than mode A (better): median diff -6.71 %, 95% CI [-15.2, -1.63], r_rb = -0.85, Holm p = 0.041.
- Time in contact: no significant C vs A difference (median diff -4.64 %, Holm p = 0.16).
- Time in contact: mode C is higher than mode B (worse): median diff +3.58 %, 95% CI [+0.382, +6.05], r_rb = +0.85, Holm p = 0.041.
- Longitudinal RMS jerk: mode B is lower than mode A (better): median diff -1.88 m/s^3, 95% CI [-2.75, -1.27], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: mode C is lower than mode A (better): median diff -1.71 m/s^3, 95% CI [-2.49, -0.975], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: no significant C vs B difference (median diff +0.271 m/s^3, Holm p = 0.322).
- Transverse RMS jerk: no significant B vs A difference (median diff -2.4 m/s^3, Holm p = 0.082).
- Transverse RMS jerk: no significant C vs A difference (median diff -1.53 m/s^3, Holm p = 0.082).
- Transverse RMS jerk: no significant C vs B difference (median diff +0.00371 m/s^3, Holm p = 0.77).
- Mean applied damping: mode B is higher than mode A: median diff +13 N*s/m, 95% CI [+13, +13], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is higher than mode A: median diff +10.2 N*s/m, 95% CI [+9.64, +11.1], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is lower than mode B: median diff -2.8 N*s/m, 95% CI [-3.36, -1.87], r_rb = -1.00, Holm p = 0.00586.
