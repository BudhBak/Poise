# Desktop human-in-the-loop sessions (empty on purpose)

Sessions recorded with `telemetry/scripts/hil_trackpad_bench.py` land here as `<session>/`:
`trial_mode_<M>_run_<block>_hitl.csv` (same schema as the SIL CSVs), `manifest.json`, `session.json`
(OS-entropy block order, blinding flag, validity rule, environment).

This is a real human driving the firmware control law on a SIMULATED device through a mouse/trackpad/pen.
It is not hardware-in-the-loop. No session has been recorded yet; nothing here may be synthesised.

Evaluate a session with the pre-registered validity rule:
    python telemetry/scripts/evaluator.py --data telemetry/data_hil/<session> --min-mt 1.0 --max-speed 0.25
