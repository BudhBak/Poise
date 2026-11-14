# Evaluation: `telemetry\data_sil\null_grip`

- Trials analysed: 30 (30 completed the 180 mm path); excluded by validity rule: 0
- Generator: firmware/sim/test_bench.cpp, grip model: null, Schmidt k: 0.04 s, seed: 2026
- Steering-law path length L = 182.58 mm, W = 2.70 mm, ID = L/W = 67.62 bit
- Passivity-controller active samples (all trials): 0
- Note: simulation runs are random seeds, not participants. p-values describe robustness to the simulated noise, not population-level inference.

| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |
|---|---|---|---|---|---|---|---|
| Contacts [count] | 0.8 ± 2.2 | 0 ± 0 | 0.9 ± 1.7 | 0.174 | +0 (1) | +0 (1) | +0 (0.75) |
| Movement time [s] | 3.65 ± 0.33 | 5.04 ± 0.33 | 4.19 ± 0.39 | 0.000112 | +1.51 (0.00586) | +0.572 (0.0488) | -0.897 (0.00586) |
| Steering throughput (ID=L/W) [bit/s] | 18.6 ± 1.6 | 13.5 ± 0.88 | 16.3 ± 1.5 | 0.000112 | -5.63 (0.00586) | -2.71 (0.0371) | +3.12 (0.00586) |
| Lateral error SD [mm] | 0.498 ± 0.26 | 0.354 ± 0.074 | 0.461 ± 0.11 | 0.00123 | -0.0831 (0.00781) | +0.00787 (0.557) | +0.0826 (0.00586) |
| Effective steering throughput (W_e) [bit/s] | 28.3 ± 9.1 | 26 ± 6.5 | 24.5 ± 6.7 | 0.045 | -3.82 (0.32) | -5.07 (0.252) | -0.618 (0.322) |
| Time in contact [%] | 7.31 ± 23 | 0 ± 0 | 1.12 ± 2.5 | 0.368 | +0 (1) | +0 (1) | +0 (1) |
| Longitudinal RMS jerk [m/s^3] | 6.6 ± 1.4 | 5.62 ± 1.2 | 5.59 ± 1.2 | 0.0005 | -0.948 (0.00586) | -0.893 (0.00586) | -0.00164 (0.492) |
| Transverse RMS jerk [m/s^3] | 11.3 ± 2.5 | 10 ± 1.6 | 10.6 ± 1.5 | 0.0273 | -0.663 (0.0293) | -0.367 (0.129) | +0.538 (0.129) |
| Mean applied damping [N*s/m] | 0 ± 0 | 13 ± 0 | 4.84 ± 0.32 | 4.54e-05 | +13 (0.00586) | +4.76 (0.00586) | -8.24 (0.00586) |

Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.

## Verdicts

- Contacts: no significant B vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs A difference (median diff +0 count, Holm p = 1).
- Contacts: no significant C vs B difference (median diff +0 count, Holm p = 0.75).
- Movement time: mode B is higher than mode A (worse): median diff +1.51 s, 95% CI [+1.33, +1.64], r_rb = +1.00, Holm p = 0.00586.
- Movement time: mode C is higher than mode A (worse): median diff +0.572 s, 95% CI [+0.534, +0.782], r_rb = +0.71, Holm p = 0.0488.
- Movement time: mode C is lower than mode B (better): median diff -0.897 s, 95% CI [-0.988, -0.692], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode B is lower than mode A (worse): median diff -5.63 bit/s, 95% CI [-6.13, -5], r_rb = -1.00, Holm p = 0.00586.
- Steering throughput (ID=L/W): mode C is lower than mode A (worse): median diff -2.71 bit/s, 95% CI [-3.34, -2.36], r_rb = -0.75, Holm p = 0.0371.
- Steering throughput (ID=L/W): mode C is higher than mode B (better): median diff +3.12 bit/s, 95% CI [+2.24, +3.27], r_rb = +1.00, Holm p = 0.00586.
- Lateral error SD: mode B is lower than mode A (better): median diff -0.0831 mm, 95% CI [-0.107, -0.0438], r_rb = -0.96, Holm p = 0.00781.
- Lateral error SD: no significant C vs A difference (median diff +0.00787 mm, Holm p = 0.557).
- Lateral error SD: mode C is higher than mode B (worse): median diff +0.0826 mm, 95% CI [+0.0529, +0.196], r_rb = +1.00, Holm p = 0.00586.
- Effective steering throughput (W_e): no significant B vs A difference (median diff -3.82 bit/s, Holm p = 0.32).
- Effective steering throughput (W_e): no significant C vs A difference (median diff -5.07 bit/s, Holm p = 0.252).
- Effective steering throughput (W_e): no significant C vs B difference (median diff -0.618 bit/s, Holm p = 0.322).
- Time in contact: no significant B vs A difference (median diff +0 %, Holm p = 1).
- Time in contact: no significant C vs A difference (median diff +0 %, Holm p = 1).
- Time in contact: no significant C vs B difference (median diff +0 %, Holm p = 1).
- Longitudinal RMS jerk: mode B is lower than mode A (better): median diff -0.948 m/s^3, 95% CI [-1.04, -0.823], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: mode C is lower than mode A (better): median diff -0.893 m/s^3, 95% CI [-1.2, -0.8], r_rb = -1.00, Holm p = 0.00586.
- Longitudinal RMS jerk: no significant C vs B difference (median diff -0.00164 m/s^3, Holm p = 0.492).
- Transverse RMS jerk: mode B is lower than mode A (better): median diff -0.663 m/s^3, 95% CI [-1.88, -0.0737], r_rb = -0.89, Holm p = 0.0293.
- Transverse RMS jerk: no significant C vs A difference (median diff -0.367 m/s^3, Holm p = 0.129).
- Transverse RMS jerk: no significant C vs B difference (median diff +0.538 m/s^3, Holm p = 0.129).
- Mean applied damping: mode B is higher than mode A: median diff +13 N*s/m, 95% CI [+13, +13], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is higher than mode A: median diff +4.76 N*s/m, 95% CI [+4.61, +5.02], r_rb = +1.00, Holm p = 0.00586.
- Mean applied damping: mode C is lower than mode B: median diff -8.24 N*s/m, 95% CI [-8.39, -7.98], r_rb = -1.00, Holm p = 0.00586.
