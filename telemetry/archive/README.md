# Legacy v1 data: kept for transparency, INVALID for any conclusion

These files were produced by the v1 code and are retained only so the history of the project is auditable.
Do not analyse or cite them. Reasons (details in the workspace audit `JUDGE_REVIEW.md`, finding P0-4 and criterion 3.2):

| Folder | Why it is invalid |
|---|---|
| `legacy_v1/sil/` (30 CSV) | The v1 simulated grip signal was computed from the ground-truth wire curvature and the true lateral error (v1 `test_bench.cpp:86-95`). The controller therefore saw the task geometry through its sensor: an oracle. The v1 sim also bypassed `HapticEngine::step()` (no encoder quantisation, deadband, saturation or motor model). These files were byte-identical to the v1 `raw_data/` copy, so 30 duplicates were removed after a `cmp` check. |
| `legacy_v1/hil/` (15 CSV) | Despite the name, no hardware was in the loop: a browser mouse simulation with a virtual spring. The "blinded" protocol showed live damping on screen, trials lasted 0.12-0.54 s, and every trial started at difficulty 0.28 due to a `setMode()` state leak. |
| `legacy_v1/figures/` | `hil_benchmark_v1.png` was produced by no script in the repository; `sil_benchmark_v1.png` was a duplicate of another figure. |

All v2 results are regenerated from source by `telemetry/scripts/run_pipeline.py` into `telemetry/data_sil/` and `telemetry/results/`.
