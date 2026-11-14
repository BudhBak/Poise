# Hardware trials (empty: the hardware does not exist yet)

`telemetry/scripts/serial_logger.py --port <COMx> --mode <A|B|C> --run <N>` writes, per trial,
`<session>/trial_mode_<M>_run_<N>_hw.csv` (firmware header, 25 Hz) and a `.log` with every `#`-status line
(#SELFTEST, #POLARITY, #STAT with loop overruns and telemetry drops). Malformed rows are counted and dropped.

Hardware CSVs contain only what the board measures (no lateral error, so lateral-SD metrics are NaN; contacts
come from the wire-loop input). Evaluate with `evaluator.py --data telemetry/data_hw/<session>`.

Note: a hardware CSV spans the whole logging window, including idle time before the hand moves. Trim each file to the movement before evaluation, and record the trim rule in the session notes.
