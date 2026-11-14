"""SymbioGrip benchmark evaluator.

Computes per-trial task metrics and paired, multiplicity-corrected statistics for modes A/B/C from
SIL (firmware/sim/test_bench), desktop-bench or hardware logs that follow the firmware CSV schema.

Metrics per trial
  contacts            debounced wall/wire contacts (count)
  mt_s                movement time
  speed_mps           mean progress speed
  tp_nominal_bps      Accot-Zhai steering throughput, ID = L/W (L = wire arc length, W = 2.70 mm), TP = ID/MT
  sd_lat_mm           SD of lateral error (sim / bench only)
  tp_effective_bps    steering throughput with W_e = 4.133 * SD_lat
  contact_time_pct    % of samples with |lateral error| >= corridor half-width (sim / bench only)
  jerk_long_rms       RMS jerk along the rail (Savitzky-Golay 3rd derivative of position), m/s^3
  jerk_trans_rms      RMS jerk across the rail (stylus y), m/s^3 (sim / bench only)
  mean_damping        mean applied damping (N*s/m)
  pc_samples          logged samples with passivity-controller damping > 0

Statistics per metric
  Friedman test across A/B/C; pairwise Wilcoxon signed-rank tests (A-B, A-C, B-C) with Holm-Bonferroni
  correction within the metric; matched-pairs rank-biserial correlation; median paired difference with a
  bootstrap 95 % CI (10,000 resamples, fixed seed).

Usage
  python evaluator.py                                  # evaluates telemetry/data_sil/causal
  python evaluator.py --data <dir> [--data <dir> ...] [--out-root <dir>] [--min-mt S] [--max-speed V]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats
from scipy.signal import savgol_filter

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_DATA = REPO_ROOT / "telemetry" / "data_sil" / "causal"
RESULTS_ROOT = REPO_ROOT / "telemetry" / "results"

DEFAULT_GEOMETRY = {"track_len_m": 0.180, "corridor_half_width_m": 0.00135,
                    "wire_amplitude_m": 0.0023, "wire_wavelength_m": 0.060}
DEFAULT_COUNTS_PER_METER = 60000.0
MODES = ("A", "B", "C")
PAIRS = (("A", "B"), ("A", "C"), ("B", "C"))
MODE_LABELS = {"A": "A unassisted", "B": "B fixed 13 N*s/m", "C": "C adaptive"}

METRICS = [
    # key, label, unit, better ("lower"/"higher"/None)
    ("contacts", "Contacts", "count", "lower"),
    ("mt_s", "Movement time", "s", "lower"),
    ("tp_nominal_bps", "Steering throughput (ID=L/W)", "bit/s", "higher"),
    ("sd_lat_mm", "Lateral error SD", "mm", "lower"),
    ("tp_effective_bps", "Effective steering throughput (W_e)", "bit/s", "higher"),
    ("contact_time_pct", "Time in contact", "%", "lower"),
    ("jerk_long_rms", "Longitudinal RMS jerk", "m/s^3", "lower"),
    ("jerk_trans_rms", "Transverse RMS jerk", "m/s^3", "lower"),
    ("mean_damping", "Mean applied damping", "N*s/m", None),
]

NAME_RE = re.compile(r"trial_mode_([ABC])_run_(\d+)_")


def wire_arc_length(geo: dict) -> float:
    a, lam, L = geo["wire_amplitude_m"], geo["wire_wavelength_m"], geo["track_len_m"]
    k = 2 * math.pi / lam
    x = np.linspace(0.0, L, 20001)
    return float(np.trapezoid(np.sqrt(1.0 + (a * k * np.cos(k * x)) ** 2), x))


def _rms_jerk(signal: np.ndarray, dt: float) -> float:
    n = len(signal)
    win = int(round(0.11 / dt)) | 1          # ~110 ms window, odd
    win = max(win, 7)
    if n < win + 4:
        return float("nan")
    j = savgol_filter(signal, window_length=win, polyorder=4, deriv=3, delta=dt)
    edge = win // 2
    return float(np.sqrt(np.mean(j[edge:-edge] ** 2)))


def load_manifest(data_dir: Path) -> dict:
    p = data_dir / "manifest.json"
    return json.loads(p.read_text()) if p.exists() else {}


def trial_metrics(path: Path, geo: dict, counts_per_meter: float, path_len: float) -> dict | None:
    m = NAME_RE.search(path.name)
    if not m:
        return None
    df = pd.read_csv(path, comment="#")
    rec = frame_metrics(df, geo, counts_per_meter, path_len)
    if rec is None:
        return None
    rec.update({"file": path.name, "mode": m.group(1), "run": int(m.group(2))})
    return rec


def frame_metrics(df: pd.DataFrame, geo: dict, counts_per_meter: float, path_len: float) -> dict | None:
    """Per-trial metrics from a telemetry frame (shared with analyze_innovations.py)."""
    for c in df.columns:
        if c != "mode":
            df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_ms"])
    if len(df) < 5:
        return None
    t = df["t_ms"].to_numpy(float) / 1000.0
    dt = float(np.median(np.diff(t)))
    pos = df["pos_m"].to_numpy(float) if "pos_m" in df else df["enc_counts"].to_numpy(float) / counts_per_meter
    mt = float(t[-1] - t[0] + dt)
    progress = float(pos[-1] - pos[0])
    W = 2.0 * geo["corridor_half_width_m"]
    rec = {
        "samples": len(df), "log_dt_s": dt,
        "completed": bool(pos[-1] >= geo["track_len_m"] - 0.001),
        "contacts": float(df["contacts"].iloc[-1]),
        "mt_s": mt,
        "speed_mps": progress / mt if mt > 0 else float("nan"),
        "tp_nominal_bps": (path_len / W) / mt,
        "jerk_long_rms": _rms_jerk(pos, dt),
        "mean_damping": float(df["damping_Nspm"].mean()),
        "pc_samples": int((df["pc_beta"] > 0).sum()) if "pc_beta" in df else 0,
    }
    if "lat_err_m" in df:
        lat = df["lat_err_m"].to_numpy(float)
        sd = float(np.std(lat, ddof=1))
        rec["sd_lat_mm"] = sd * 1000.0
        rec["tp_effective_bps"] = (path_len / (4.133 * sd)) / mt if sd > 0 else float("nan")
        rec["contact_time_pct"] = 100.0 * float(np.mean(np.abs(lat) >= geo["corridor_half_width_m"]))
    else:
        rec["sd_lat_mm"] = rec["tp_effective_bps"] = rec["contact_time_pct"] = float("nan")
    rec["jerk_trans_rms"] = _rms_jerk(df["y_m"].to_numpy(float), dt) if "y_m" in df else float("nan")
    return rec


def load_trials(data_dir: Path) -> tuple[pd.DataFrame, dict]:
    man = load_manifest(data_dir)
    geo = dict(DEFAULT_GEOMETRY)
    plant = man.get("plant", {})
    for k in geo:
        if k in plant:
            geo[k] = float(plant[k])
    cpm = float(man.get("counts_per_meter", DEFAULT_COUNTS_PER_METER))
    L = wire_arc_length(geo)
    recs = [r for p in sorted(data_dir.glob("trial_mode_*.csv")) if (r := trial_metrics(p, geo, cpm, L))]
    info = {"geometry": geo, "path_length_m": L, "counts_per_meter": cpm, "manifest": {k: v for k, v in man.items() if k != "trials"}}
    return pd.DataFrame(recs), info


def rank_biserial(d: np.ndarray) -> float:
    d = d[d != 0]
    if len(d) == 0:
        return 0.0
    r = stats.rankdata(np.abs(d))
    rp, rm = r[d > 0].sum(), r[d < 0].sum()
    return float((rp - rm) / (rp + rm))


def bootstrap_median_ci(d: np.ndarray, n_boot: int = 10000, seed: int = 2026) -> tuple[float, float]:
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(d), size=(n_boot, len(d)))
    meds = np.median(d[idx], axis=1)
    return float(np.percentile(meds, 2.5)), float(np.percentile(meds, 97.5))


def holm(pvals: list[float]) -> list[float]:
    order = np.argsort(pvals)
    m = len(pvals)
    adj = [0.0] * m
    running = 0.0
    for rank, i in enumerate(order):
        val = min(1.0, (m - rank) * pvals[i])
        running = max(running, val)
        adj[i] = running
    return adj


def paired_stats(trials: pd.DataFrame, metric: str) -> dict | None:
    piv = trials.pivot_table(index="run", columns="mode", values=metric, aggfunc="first")
    if not set(MODES).issubset(piv.columns):
        return None
    piv = piv[list(MODES)].dropna()
    n = len(piv)
    if n < 3:
        return None
    out = {"n_runs": n, "mean": {m: float(piv[m].mean()) for m in MODES},
           "sd": {m: float(piv[m].std(ddof=1)) for m in MODES},
           "median": {m: float(piv[m].median()) for m in MODES}}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if np.allclose(piv.values, piv.values[:, :1]):
            out["friedman"] = {"chi2": 0.0, "p": 1.0}
        else:
            fr = stats.friedmanchisquare(*(piv[m] for m in MODES))
            out["friedman"] = {"chi2": float(fr.statistic), "p": float(fr.pvalue)}
        pairs = []
        for a, b in PAIRS:
            d = (piv[b] - piv[a]).to_numpy(float)
            if np.allclose(d, 0):
                w, p, method = 0.0, 1.0, "all-zero"
            else:
                res = stats.wilcoxon(piv[b], piv[a], zero_method="wilcox", method="auto")
                w, p = float(res.statistic), float(res.pvalue)
                method = "auto"
            lo, hi = bootstrap_median_ci(d)
            pairs.append({"pair": f"{b}-{a}", "W": w, "p": p, "method": method,
                          "median_diff": float(np.median(d)), "ci95": [lo, hi],
                          "rank_biserial": rank_biserial(d), "n_nonzero": int(np.sum(d != 0))})
    for pr, ph in zip(pairs, holm([pr["p"] for pr in pairs])):
        pr["p_holm"] = ph
    out["pairs"] = pairs
    return out


def verdict(label: str, unit: str, better: str | None, pr: dict) -> str:
    b, a = pr["pair"].split("-")
    if pr["p_holm"] >= 0.05:
        return (f"{label}: no significant {MODE_LABELS[b].split()[0]} vs {MODE_LABELS[a].split()[0]} difference "
                f"(median diff {pr['median_diff']:+.3g} {unit}, Holm p = {pr['p_holm']:.3g}).")
    direction = "higher" if pr["median_diff"] > 0 else "lower"
    judgement = ""
    if better:
        good = (direction == better)
        judgement = " (better)" if good else " (worse)"
    return (f"{label}: mode {b} is {direction} than mode {a}{judgement}: median diff {pr['median_diff']:+.3g} {unit}, "
            f"95% CI [{pr['ci95'][0]:+.3g}, {pr['ci95'][1]:+.3g}], r_rb = {pr['rank_biserial']:+.2f}, Holm p = {pr['p_holm']:.3g}.")


def evaluate(data_dir: Path, out_root: Path, min_mt: float, max_speed: float, quiet: bool = False) -> dict:
    trials, info = load_trials(data_dir)
    if trials.empty:
        raise SystemExit(f"No trial CSV files found in {data_dir}")
    valid = (trials["mt_s"] >= min_mt) & (trials["speed_mps"] <= max_speed)
    excluded = trials.loc[~valid, ["file", "mt_s", "speed_mps"]].to_dict("records")
    trials = trials[valid].copy()

    results = {"dataset": str(data_dir.relative_to(REPO_ROOT)) if data_dir.is_relative_to(REPO_ROOT) else str(data_dir),
               "info": info, "validity_rule": {"min_mt_s": min_mt, "max_speed_mps": max_speed},
               "n_trials": int(len(trials)), "n_excluded": len(excluded), "excluded": excluded,
               "n_completed": int(trials["completed"].sum()),
               "pc_samples_total": int(trials["pc_samples"].sum()), "metrics": {}, "verdicts": []}
    for key, label, unit, better in METRICS:
        if trials[key].isna().all():
            continue
        st = paired_stats(trials, key)
        if st is None:
            continue
        st.update({"label": label, "unit": unit, "better": better})
        results["metrics"][key] = st
        for pr in st["pairs"]:
            results["verdicts"].append(verdict(label, unit, better, pr))

    name = data_dir.name if data_dir.parent.name in ("data_sil", "data_hil", "data_hw") else f"{data_dir.parent.name}_{data_dir.name}"
    out_dir = out_root / name
    out_dir.mkdir(parents=True, exist_ok=True)
    trials.sort_values(["run", "mode"]).to_csv(out_dir / "per_trial_metrics.csv", index=False)
    (out_dir / "evaluation.json").write_text(json.dumps(results, indent=2))
    md = render_markdown(results)
    (out_dir / "evaluation.md").write_text(md, encoding="utf-8")
    results["out_dir"] = str(out_dir)
    if not quiet:
        print(md)
        print(f"[evaluator] wrote {out_dir}")
    return results


def render_markdown(r: dict) -> str:
    man = r["info"]["manifest"]
    lines = [f"# Evaluation: `{r['dataset']}`", ""]
    lines.append(f"- Trials analysed: {r['n_trials']} ({r['n_completed']} completed the 180 mm path); excluded by validity rule: {r['n_excluded']}")
    if man:
        lines.append(f"- Generator: {man.get('generator', '?')}, grip model: {man.get('grip_model', '?')}, "
                     f"Schmidt k: {man.get('schmidt_k_s', '?')} s, seed: {man.get('seed', '?')}")
    lines.append(f"- Steering-law path length L = {r['info']['path_length_m'] * 1000:.2f} mm, W = "
                 f"{2000 * r['info']['geometry']['corridor_half_width_m']:.2f} mm, ID = L/W = "
                 f"{r['info']['path_length_m'] / (2 * r['info']['geometry']['corridor_half_width_m']):.2f} bit")
    lines.append(f"- Passivity-controller active samples (all trials): {r['pc_samples_total']}")
    lines.append("- Note: simulation runs are random seeds, not participants. p-values describe robustness to the "
                 "simulated noise, not population-level inference.")
    lines += ["", "| Metric | A mean ± SD | B mean ± SD | C mean ± SD | Friedman p | B−A (Holm p) | C−A (Holm p) | C−B (Holm p) |",
              "|---|---|---|---|---|---|---|---|"]
    for key, st in r["metrics"].items():
        cells = [f"{st['mean'][m]:.3g} ± {st['sd'][m]:.2g}" for m in MODES]
        pairs = {p["pair"]: p for p in st["pairs"]}
        pc = [f"{pairs[k]['median_diff']:+.3g} ({pairs[k]['p_holm']:.3g})" for k in ("B-A", "C-A", "C-B")]
        lines.append(f"| {st['label']} [{st['unit']}] | " + " | ".join(cells) + f" | {st['friedman']['p']:.3g} | " + " | ".join(pc) + " |")
    lines += ["", "Pair cells show the median paired difference, with the Holm-corrected Wilcoxon p in parentheses.", "", "## Verdicts", ""]
    lines += [f"- {v}" for v in r["verdicts"]]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):          # Windows consoles default to cp1252; reports contain U+2212
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", action="append", type=Path, help="dataset directory (repeatable)")
    ap.add_argument("--out-root", type=Path, default=RESULTS_ROOT)
    ap.add_argument("--min-mt", type=float, default=0.0, help="validity: minimum movement time (s)")
    ap.add_argument("--max-speed", type=float, default=float("inf"), help="validity: maximum mean speed (m/s)")
    args = ap.parse_args(argv)
    for d in args.data or [DEFAULT_DATA]:
        evaluate(d.resolve(), args.out_root.resolve(), args.min_mt, args.max_speed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
