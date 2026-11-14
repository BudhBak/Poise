"""Log one hardware trial from the SymbioGrip firmware (UNO R4 WiFi) to telemetry/data_hw/<session>/.

The firmware (firmware/src/src.ino) prints:
  - '#'-prefixed status lines (#SELFTEST, #POLARITY, #STAT, #MODE, #FAULT ...)  -> <trial>.log
  - CSV rows with the exact header below at 25 Hz (TELEMETRY_DIV = 20)          -> trial_mode_<M>_run_<N>_hw.csv
Rows whose field count or types do not match the header are counted and dropped, never repaired.

Usage:
  python serial_logger.py --port COM5 --mode C --run 3 [--baud 115200] [--session NAME] [--seconds 60]
Press Ctrl+C to end the trial; the firmware is returned to mode A (motor coasting) on exit.
Requires: pyserial.
"""
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HEADER = "t_ms,mode,grip_adc,enc_counts,vel_mps,damping_Nspm,difficulty,tremor_rms,duty,energy_mJ,contacts"
TYPES = (int, str, int, int, float, float, float, float, float, float, int)


def parse_row(line: str) -> list | None:
    parts = line.split(",")
    if len(parts) != len(TYPES):
        return None
    try:
        vals = [t(p) for t, p in zip(TYPES, parts)]
    except ValueError:
        return None
    return vals if vals[1] in ("A", "B", "C") else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--mode", required=True, choices=list("ABCabc"))
    ap.add_argument("--run", required=True, type=int)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--session", default=datetime.now(timezone.utc).strftime("hw_%Y%m%d"))
    ap.add_argument("--seconds", type=float, default=0.0, help="stop automatically after N s (0 = Ctrl+C)")
    args = ap.parse_args(argv)
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial is required: pip install pyserial", file=sys.stderr)
        return 2

    mode = args.mode.upper()
    out = REPO / "telemetry" / "data_hw" / args.session
    out.mkdir(parents=True, exist_ok=True)
    stem = f"trial_mode_{mode}_run_{args.run}_hw"
    csv_path, log_path = out / f"{stem}.csv", out / f"{stem}.log"
    if csv_path.exists():
        print(f"refusing to overwrite {csv_path}", file=sys.stderr)
        return 2

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(2.0)                 # board resets when the port opens
    ser.reset_input_buffer()
    ser.write(mode.encode("ascii"))
    print(f"mode {mode} requested; logging to {csv_path}")
    rows = dropped = 0
    header_seen = False
    t_end = time.monotonic() + args.seconds if args.seconds > 0 else float("inf")
    with open(csv_path, "w", encoding="utf-8", newline="\n") as fc, open(log_path, "w", encoding="utf-8") as fl:
        fc.write(HEADER + "\n")
        try:
            while time.monotonic() < t_end:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    fl.write(f"{time.time():.3f} {line}\n")
                    print(line)
                    continue
                if line == HEADER:
                    header_seen = True
                    continue
                vals = parse_row(line)
                if vals is None:
                    dropped += 1
                    continue
                if vals[1] != mode:
                    fl.write(f"{time.time():.3f} # logger: row in mode {vals[1]} ignored (requested {mode})\n")
                    continue
                fc.write(line + "\n")
                rows += 1
        except KeyboardInterrupt:
            pass
        finally:
            ser.write(b"A")
            ser.close()
            fl.write(f"{time.time():.3f} # logger: rows={rows} dropped={dropped} header_seen={header_seen}\n")
    print(f"done: {rows} rows, {dropped} malformed rows dropped; firmware returned to mode A")
    return 0


if __name__ == "__main__":
    sys.exit(main())
