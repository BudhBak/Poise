"""SymbioGrip v3 innovation analysis: candidates vs. v2 Mode C and the fixed-damper family.

Reads the lab output written by firmware/sim/innovation_lab.cpp (default telemetry/data_lab/) and the ARM cycle
budget (telemetry/results/innovation/arm_cycles.json), applies the promotion rule pre-registered in
docs/InnovationResearch.md section 8, and writes:

  telemetry/results/innovation/innovation.json     every number below
  telemetry/results/figures/fig_pareto_v3.png/pdf   speed-accuracy and speed-smoothness planes (suite S1)
  InnovationReport.md                              generated report (every verdict computed here)

Statistics: per suite, paired two-sided Wilcoxon signed-rank (common random numbers, N seeds), Holm correction
within each family: primary = {MT, lateral SD, contacts} x candidates x {C_v2, B8.22}; secondary = {throughput
(nominal, effective), longitudinal jerk, transverse jerk, 6-14 Hz displacement} x candidates x {C_v2, B8.22};
frontier = {lateral SD, contacts, longitudinal jerk} x candidates (residual vs. the per-seed fixed-damper curve).

Usage:  python analyze_innovations.py [--lab telemetry/data_lab] [--report InnovationReport.md]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal, stats

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import evaluator as ev  # noqa: E402

REPO = SCRIPT_DIR.parents[1]
FS_CTRL = 500.0
T = 1.0 / FS_CTRL
NAME_RE = re.compile(r"^trial_(.+)_run_(\d+)\.csv$")
FIXED = ["A", "B4.5", "B8.22", "B13", "B20"]
BASELINES = FIXED + ["C_v2"]
COMPARATORS = ["C_v2", "B8.22"]
PRIMARY = [("mt_s", "Movement time", "s", "lower"), ("sd_lat_mm", "Lateral SD", "mm", "lower"),
           ("contacts", "Contacts", "count", "lower")]
SECONDARY = [("tp_nominal_bps", "Throughput (ID/MT)", "bit/s", "higher"),
             ("tp_effective_bps", "Throughput (W_e)", "bit/s", "higher"),
             ("jerk_long_rms", "Longitudinal jerk", "m/s^3", "lower"),
             ("jerk_trans_rms", "Transverse jerk", "m/s^3", "lower"),
             ("disp_tr_um", "6-14 Hz displacement", "um", "lower")]
FRONTIER = [("sd_lat_mm", "Lateral SD"), ("contacts", "Contacts"), ("jerk_long_rms", "Longitudinal jerk")]
SUITES = [("S1", "S1"), ("S2a", "S2a"), ("S2b", "S2b"), ("S3", "S3"),
          ("S4_rest_1.5", "S4/rest_1.5"), ("S4_rest_3.0", "S4/rest_3.0"),
          ("S4_rest_4.5", "S4/rest_4.5"), ("S4_rest_6.0", "S4/rest_6.0"), ("S5", "S5")]
SUITE_DESC = {
    "S1": "primary: causal operator, Schmidt k = 0.04 s",
    "S2a": "high command noise, k = 0.08 s",
    "S2b": "enhanced physiological tremor, tremor force x4 (x and y)",
    "S3": "firm confident grip: FSR reads steady 8.0 N, operator tremor scale 0.5",
    "S4_rest_1.5": "resting grip 1.5 N (sensor offset -0.5 N)", "S4_rest_3.0": "resting grip 3.0 N (offset +1.0 N)",
    "S4_rest_4.5": "resting grip 4.5 N (offset +2.5 N)", "S4_rest_6.0": "resting grip 6.0 N (offset +4.0 N)",
    "S5": "breathing-cue swell and dither non-interference",
}
ALPHA = 0.05
# Pre-registered predictions (copied verbatim from docs/InnovationResearch.md section 9; not evaluated by hand).
PREDICTIONS = [
    ("P1", "No candidate breaks the speed-accuracy frontier: |lateral-SD frontier residual| <= ~2 % and not significant.",
     "frontier_sd"),
    ("P2", "Dual-band candidates (DB_BMFLC, DB_LTI, DB_ADAPT) reduce longitudinal jerk below the fixed-damper curve "
           "(negative jerk frontier residual).", "frontier_jerk_db"),
    ("P3", "FO_SUB is worse than FO_SUPER on longitudinal jerk.", "fo_order"),
    ("P4", "Dither (2.5 % duty) increases longitudinal jerk vs. DB_ADAPT (S5).", "dither"),
    ("P5", "v2 Mode C is slower than in S1 under a firm 8 N grip (S3) and at a 6 N resting grip (S4), "
           "relative to B8.22.", "c_slows"),
]


# ------------------------------------------------------------------------------------------------ metrics
def band_displacement_um(df: pd.DataFrame, lo: float = 6.0, hi: float = 14.0) -> float:
    t = df["t_ms"].to_numpy(float) / 1000.0
    x = df["pos_m"].to_numpy(float)
    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    if len(x) < int(2 * fs) or hi >= 0.5 * fs:
        return float("nan")
    sos = signal.butter(4, [lo, hi], btype="bandpass", fs=fs, output="sos")
    y = signal.sosfiltfilt(sos, x)
    edge = int(0.5 * fs)
    return float(np.sqrt(np.mean(y[edge:-edge] ** 2)) * 1e6)


def load_suite(d: Path) -> tuple[pd.DataFrame, dict]:
    man = json.loads((d / "manifest.json").read_text())
    geo = dict(ev.DEFAULT_GEOMETRY)
    for k in geo:
        if k in man.get("plant", {}):
            geo[k] = float(man["plant"][k])
    cpm = float(man.get("counts_per_meter", ev.DEFAULT_COUNTS_PER_METER))
    L = ev.wire_arc_length(geo)
    recs = []
    for p in sorted(d.glob("trial_*_run_*.csv")):
        m = NAME_RE.match(p.name)
        if not m:
            continue
        df = pd.read_csv(p)
        rec = ev.frame_metrics(df.copy(), geo, cpm, L)
        if rec is None:
            continue
        rec["disp_tr_um"] = band_displacement_um(df)
        rec.update({"cond": m.group(1), "run": int(m.group(2))})
        recs.append(rec)
    comp = {(t["cond"], t["run"]): t for t in man.get("trials", [])}
    out = pd.DataFrame(recs)
    out["fault"] = [comp.get((c, r), {}).get("fault", 0) for c, r in zip(out["cond"], out["run"])]
    out["pc_activations"] = [comp.get((c, r), {}).get("pc_activations", 0) for c, r in zip(out["cond"], out["run"])]
    return out, man


# ------------------------------------------------------------------------------------------------ statistics
def wilcoxon(d: np.ndarray) -> float:
    d = d[np.isfinite(d)]
    if len(d) < 3 or np.allclose(d, 0):
        return 1.0
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return float(stats.wilcoxon(d, zero_method="wilcox", method="auto").pvalue)


def paired(trials: pd.DataFrame, metric: str, a: str, b: str) -> dict:
    piv = trials.pivot_table(index="run", columns="cond", values=metric, aggfunc="first")
    if a not in piv or b not in piv:
        return {}
    d = (piv[b] - piv[a]).dropna().to_numpy(float)
    lo, hi = ev.bootstrap_median_ci(d) if len(d) >= 3 else (float("nan"), float("nan"))
    ref = float(piv[a].median())
    return {"n": int(len(d)), "median_diff": float(np.median(d)), "ci95": [lo, hi], "p": wilcoxon(d),
            "rel_pct": 100.0 * float(np.median(d)) / ref if ref else float("nan"),
            "median_a": ref, "median_b": float(piv[b].median())}


def apply_holm(entries: list[dict]) -> None:
    if not entries:
        return
    for e, ph in zip(entries, ev.holm([e["p"] for e in entries])):
        e["p_holm"] = ph


def frontier_residuals(trials: pd.DataFrame, cand: str, metric: str) -> dict:
    """Per seed: candidate metric minus the fixed-damper curve (A, B4.5..B20) interpolated at the candidate's MT."""
    res, rel, outside = [], [], 0
    for run, g in trials.groupby("run"):
        fx = g[g["cond"].isin(FIXED)].sort_values("mt_s")
        cr = g[g["cond"] == cand]
        if len(fx) < 2 or cr.empty:
            continue
        mt = float(cr["mt_s"].iloc[0])
        xs, ys = fx["mt_s"].to_numpy(float), fx[metric].to_numpy(float)
        if mt < xs[0] or mt > xs[-1]:
            outside += 1
        yhat = float(np.interp(mt, xs, ys))
        r = float(cr[metric].iloc[0]) - yhat
        res.append(r)
        rel.append(100.0 * r / yhat if abs(yhat) > 1e-9 else float("nan"))
    res, rel = np.array(res), np.array(rel)
    lo, hi = ev.bootstrap_median_ci(res) if len(res) >= 3 else (float("nan"), float("nan"))
    rlo, rhi = ev.bootstrap_median_ci(rel[np.isfinite(rel)]) if np.isfinite(rel).sum() >= 3 else (float("nan"), float("nan"))
    return {"n": int(len(res)), "median": float(np.median(res)) if len(res) else float("nan"), "ci95": [lo, hi],
            "rel_ci95": [rlo, rhi],
            "median_rel_pct": float(np.nanmedian(rel)) if np.isfinite(rel).any() else float("nan"), "p": wilcoxon(res),
            "seeds_outside_fixed_mt_range": int(outside)}


def analyse_suite(trials: pd.DataFrame) -> dict:
    conds = sorted(trials["cond"].unique(), key=lambda c: (c not in BASELINES, BASELINES.index(c) if c in BASELINES else 0, c))
    cands = [c for c in conds if c not in BASELINES]
    metrics = PRIMARY + SECONDARY
    medians = {c: {m: float(trials[trials["cond"] == c][m].median()) for m, *_ in metrics + [("mean_damping",)]}
               for c in conds}
    integrity = {c: {"completed": int(trials[trials["cond"] == c]["completed"].sum()),
                     "n": int((trials["cond"] == c).sum()),
                     "faults": int((trials[trials["cond"] == c]["fault"] != 0).sum())} for c in conds}
    fam = {"primary": [], "secondary": []}
    tests = {}
    for c in cands:
        for comp in COMPARATORS:
            for famname, mlist in (("primary", PRIMARY), ("secondary", SECONDARY)):
                for m, label, unit, better in mlist:
                    e = paired(trials, m, comp, c)
                    if not e:
                        continue
                    e.update({"cand": c, "comp": comp, "metric": m, "label": label, "unit": unit, "better": better})
                    tests.setdefault(c, []).append(e)
                    fam[famname].append(e)
    apply_holm(fam["primary"])
    apply_holm(fam["secondary"])
    for e in fam["primary"] + fam["secondary"]:
        good = (e["median_diff"] < 0) if e["better"] == "lower" else (e["median_diff"] > 0)
        e["sig"] = e["p_holm"] < ALPHA
        e["outcome"] = ("better" if good else "worse") if e["sig"] else "n.s."
    front = []
    fr = {}
    if set(FIXED).issubset(conds):
        for c in cands + ["C_v2"]:
            for m, label in FRONTIER:
                e = frontier_residuals(trials, c, m)
                e.update({"cand": c, "metric": m, "label": label})
                fr.setdefault(c, []).append(e)
                if c != "C_v2":
                    front.append(e)
        apply_holm(front)
        for c in fr:
            for e in fr[c]:
                if "p_holm" not in e:          # C_v2 reference row, own family of 3
                    pass
        cv = fr.get("C_v2", [])
        apply_holm(cv)
        for e in front + cv:
            e["sig"] = e["p_holm"] < ALPHA
            e["outcome"] = ("below curve" if e["median"] < 0 else "above curve") if e["sig"] else "on curve (n.s.)"
        # POST-HOC (deviation, see report section 1b): Holm within each candidate's own 3 frontier tests
        for c in fr:
            for e, ph in zip(fr[c], ev.holm([e["p"] for e in fr[c]])):
                e["p_holm_cand"] = ph
                e["sig_cand"] = ph < ALPHA
    # POST-HOC: Holm within each candidate's own primary family (3 metrics x 2 comparators)
    for c in cands:
        prim = [e for e in tests.get(c, []) if e["metric"] in {m for m, *_ in PRIMARY}]
        for e, ph in zip(prim, ev.holm([e["p"] for e in prim])):
            good = (e["median_diff"] < 0) if e["better"] == "lower" else (e["median_diff"] > 0)
            e["p_holm_cand"] = ph
            e["outcome_cand"] = ("better" if good else "worse") if ph < ALPHA else "n.s."
    n_seeds = int(trials["run"].nunique())
    p_min = 2.0 / 2 ** n_seeds                  # smallest exact two-sided Wilcoxon p with n non-zero pairs
    audit = {"n_seeds": n_seeds, "min_exact_p": p_min,
             "families": {"primary": len(fam["primary"]), "secondary": len(fam["secondary"]), "frontier": len(front)}}
    audit["min_attainable_holm_p"] = {k: min(1.0, v * p_min) for k, v in audit["families"].items()}
    return {"conditions": conds, "candidates": cands, "medians": medians, "integrity": integrity,
            "tests": tests, "frontier": fr, "multiplicity": audit}


# ------------------------------------------------------------------------------------------------ passivity
def parse_header_floats(path: Path, pattern: str) -> dict[str, float]:
    return {m.group(1): float(m.group(2)) for m in re.finditer(pattern, path.read_text(encoding="utf-8"))}


def adv_checks(adv: dict) -> dict:
    cfg = parse_header_floats(REPO / "firmware" / "src" / "config.h", r"#define\s+(\w+)\s+([-+0-9.eE]+)f?")
    lp = parse_header_floats(REPO / "firmware" / "src" / "admittance.h", r"\b(LP_[AB]\d)\s*=\s*([-+0-9.eE]+)f")
    b_emf = cfg["MOTOR_KT_EFF"] ** 2 / cfg["MOTOR_R_OHM"]
    b_phys = cfg["B_FRICTION_EST"] + b_emf
    beta_max = cfg["PC_BETA_MAX"]
    f = np.array(adv["freq_hz"], float)
    th = 2 * np.pi * f * T
    z = np.exp(1j * th)
    L = (lp["LP_B0"] + lp["LP_B1"] / z + lp["LP_B2"] / z ** 2) / (1 + lp["LP_A1"] / z + lp["LP_A2"] / z ** 2)

    def b_req(G):                                   # generalised Colgate-Schenkel: -Re{e^{-j th} H(e^{j th})}
        return float(np.max(-np.real(np.exp(-1j * th) * G)))

    out = {"b_emf": b_emf, "b_phys": b_phys, "beta_max": beta_max, "grid_points": int(len(f)),
           "grid_hz": [float(f[0]), float(f[-1])], "conditions": {}}
    # reference: v2 B13 and v2 C extreme, analytic G = B L
    for name, Bs in (("B13", [13.0]), ("C_v2", [cfg["DAMPING_MIN"], cfg["DAMPING_MAX"]])):
        r = adv["conditions"].get(name, {})
        m_ff = min(b_phys - b_req(B * L - b_emf * L) for B in Bs)
        m_fb = min(b_phys - b_req(B * L + beta_max) for B in Bs)
        out["conditions"][name] = {"v2": True, "cs_margin": m_ff, "cs_margin_fallback": m_fb, **_time_checks(r)}
    for name, r in adv["conditions"].items():
        if r.get("v2"):
            continue
        c = r.get("bemf_comp", 1.0)
        margins, fallbacks, hf = [], [], []
        for d in (0, 1):
            G = np.array(r[f"G_d{d}_re"]) + 1j * np.array(r[f"G_d{d}_im"])
            margins.append(b_phys - b_req(G - c * b_emf * L))
            fallbacks.append(b_phys - b_req(G + beta_max))
            band = (f >= 6.0) & (f <= 14.0)
            low = (f >= 0.5) & (f <= 2.0)
            hf.append({"d": d, "re_G_0.5_2Hz": float(np.mean(np.real(G[low]))),
                       "re_G_6_14Hz": float(np.mean(np.real(G[band]))),
                       "min_re_G": float(np.min(np.real(G))), "f_min_re_G": float(f[np.argmin(np.real(G))])})
        out["conditions"][name] = {"v2": False, "cs_margin": float(min(margins)), "cs_margin_fallback": float(min(fallbacks)),
                                   "cs_margin_d0_d1": margins, "cs_fallback_d0_d1": fallbacks, "rendered": hf,
                                   "linearity_rel_err": r.get("linearity_rel_err"), **_time_checks(r)}
    for name, e in out["conditions"].items():
        crit = {
            "C-S margin > 0 (feed-forward path)": e["cs_margin"] > 0,
            "C-S margin > 0 (PC fallback path)": e["cs_margin_fallback"] > 0,
            "nominal closed loop work <= 0": e["nominal_work_j"] <= 0,
            "8 Hz reversal work <= 0 (nominal)": e["rev_nom_work_j"] <= 0,
            "2.5x over-comp, PC on: work <= 5 mJ": e["over_on_work_j"] <= 0.005,
            "2.5x over-comp 8 Hz reversal, PC on: work <= 5 mJ": e["rev_over_work_j"] <= 0.005,
            "quantisation-only d > 0.05 in <= 1 % of ticks": e["quant_frac_d"] <= 0.01,
        }
        if not e["v2"]:
            crit["superposition error < 1e-3 (d frozen)"] = (e["linearity_rel_err"] or 1.0) < 1e-3
        e["criteria"] = crit
        e["pass"] = all(crit.values())
    return out


def _time_checks(r: dict) -> dict:
    return {"nominal_work_j": r["nominal"]["work_j"], "over_off_work_j": r["overcomp_pc_off"]["work_j"],
            "over_on_work_j": r["overcomp_pc_on"]["work_j"], "over_on_pc": r["overcomp_pc_on"]["pc"],
            "rev_nom_work_j": r["reversal8hz_nominal"]["work_j"], "rev_over_work_j": r["reversal8hz_overcomp"]["work_j"],
            "rev_over_pc": r["reversal8hz_overcomp"]["pc"], "quant_max_env": r["quant"]["max_env"],
            "quant_max_d": r["quant"]["max_d"], "quant_frac_d": r["quant"]["frac_d_gt_0.05"],
            "latency63_s": r["latency63_s"]}


# ------------------------------------------------------------------------------------------------ promotion
def promotion(s1: dict, adv: dict, cycles: dict) -> dict:
    worst_ok = bool(cycles.get("probes", {}).get("probe_worst", {}).get("pass", False))
    out = {}
    for c in s1["candidates"]:
        fr = {e["metric"]: e for e in s1["frontier"].get(c, [])}
        a = any(fr[m]["sig"] and fr[m]["median"] < 0 for m in ("sd_lat_mm", "contacts") if m in fr)
        prim = [e for e in s1["tests"].get(c, []) if e["metric"] in {m for m, *_ in PRIMARY}]
        b = all(any(e["outcome"] == "better" for e in prim if e["comp"] == comp) and
                not any(e["outcome"] == "worse" for e in prim if e["comp"] == comp) for comp in COMPARATORS)
        adv_c = adv["conditions"].get(c)
        cc = bool(adv_c and adv_c["pass"]) and worst_ok
        # post-hoc variant: same rule with per-candidate Holm families (not used for the decision)
        a2 = any(fr[m]["sig_cand"] and fr[m]["median"] < 0 for m in ("sd_lat_mm", "contacts") if m in fr)
        b2 = all(any(e["outcome_cand"] == "better" for e in prim if e["comp"] == comp) and
                 not any(e["outcome_cand"] == "worse" for e in prim if e["comp"] == comp) for comp in COMPARATORS)
        jerk = fr.get("jerk_long_rms", {})
        out[c] = {"a_accuracy_frontier_break": a, "b_better_than_C_v2_and_B8.22": b, "c_all_checks": cc,
                  "promote": a and b and cc,
                  "posthoc": {"a": a2, "b": b2, "promote": a2 and b2 and cc,
                              "jerk_frontier_break": bool(jerk.get("sig_cand") and jerk.get("median", 0) < 0)}}
    return out


def predictions(res: dict) -> list[dict]:
    s1 = res["suites"].get("S1")
    out = []
    for pid, text, key in PREDICTIONS:
        verdict, detail = "not evaluable", ""
        if s1 and key == "frontier_sd":
            rows = [e for c in s1["candidates"] for e in s1["frontier"][c] if e["metric"] == "sd_lat_mm"]
            brk = [e["cand"] for e in rows if e["sig"]]
            mx = max(abs(e["median_rel_pct"]) for e in rows)
            verdict = "held" if not brk and mx <= 2.5 else "failed"
            detail = f"significant: {brk or 'none'}; max |median residual| = {mx:.1f} %"
        elif s1 and key == "frontier_jerk_db":
            rows = [e for c in ("DB_BMFLC", "DB_LTI", "DB_ADAPT") for e in s1["frontier"].get(c, []) if e["metric"] == "jerk_long_rms"]
            ok = [e["cand"] for e in rows if e["sig"] and e["median"] < 0]
            verdict = "held" if len(ok) == len(rows) and rows else ("partly held" if ok else "failed")
            detail = "; ".join(f"{e['cand']} {e['median_rel_pct']:+.1f} % (Holm p = {e['p_holm']:.3g})" for e in rows)
        elif s1 and key == "fo_order":
            m = s1["medians"]
            if "FO_SUB" in m and "FO_SUPER" in m:
                a_, b_ = m["FO_SUB"]["jerk_long_rms"], m["FO_SUPER"]["jerk_long_rms"]
                verdict = "held" if a_ > b_ else "failed"
                detail = f"median jerk FO_SUB {a_:.2f} vs FO_SUPER {b_:.2f} m/s^3 (descriptive)"
        elif key == "dither" and "S5" in res["suites"]:
            e = paired(res["_trials"]["S5"], "jerk_long_rms", "DB_ADAPT", "DB_ADAPT_DITHER")
            if e:
                verdict = "held" if (e["median_diff"] > 0 and e["p"] < ALPHA) else "failed"
                detail = f"median diff {e['median_diff']:+.3f} m/s^3 ({e['rel_pct']:+.1f} %), unadjusted p = {e['p']:.3g}"
        elif key == "c_slows":
            parts, held = [], True
            base = paired(res["_trials"]["S1"], "mt_s", "B8.22", "C_v2") if "S1" in res["_trials"] else None
            for sname in ("S3", "S4_rest_6.0"):
                if sname in res["_trials"] and base:
                    e = paired(res["_trials"][sname], "mt_s", "B8.22", "C_v2")
                    parts.append(f"{sname}: C-B8.22 MT {e['median_diff']:+.3f} s (p = {e['p']:.3g}) vs S1 {base['median_diff']:+.3f} s")
                    held &= e["median_diff"] > base["median_diff"] and e["p"] < ALPHA
            if parts:
                verdict = "held" if held else "failed"
                detail = "; ".join(parts)
        out.append({"id": pid, "prediction": text, "verdict": verdict, "detail": detail})
    return out


# ------------------------------------------------------------------------------------------------ figure
def figure(s1: dict, trials: pd.DataFrame, path: Path, promoted: list[str]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    C_FIX, C_V2, C_CAND, C_PROM = "#2a78d6", "#1baf7a", "#eb6834", "#7c4dc4"
    med = trials.groupby("cond")[["mt_s", "sd_lat_mm", "jerk_long_rms"]].median()
    q1 = trials.groupby("cond")[["mt_s", "sd_lat_mm", "jerk_long_rms"]].quantile(0.25)
    q3 = trials.groupby("cond")[["mt_s", "sd_lat_mm", "jerk_long_rms"]].quantile(0.75)
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 5.0), gridspec_kw={"width_ratios": [1, 1, 1.05]})
    fig.patch.set_facecolor(SURFACE)
    # candidate label column placed in each panel's empty corner (axes fraction), sorted by value -> no crossings
    bands = {"sd_lat_mm": (0.80, 0.45, 0.96), "jerk_long_rms": (0.80, 0.04, 0.50)}
    for ax, (m, lab) in zip(axes[:2], (("sd_lat_mm", "Lateral error SD (mm)"), ("jerk_long_rms", "Longitudinal RMS jerk (m/s³)"))):
        ax.set_facecolor(SURFACE)
        fx = [c for c in FIXED if c in med.index]
        ax.plot(med.loc[fx, "mt_s"], med.loc[fx, m], color=C_FIX, lw=2, zorder=2, label="fixed damper A, B4.5–B20")
        ax.scatter(med.loc[fx, "mt_s"], med.loc[fx, m], s=64, color=C_FIX, edgecolor=SURFACE, linewidth=2, zorder=3)
        for c in fx:
            ax.annotate(c, (med.loc[c, "mt_s"], med.loc[c, m]), textcoords="offset points",
                        xytext=(-4, 8) if c == "B8.22" else (6, 6), ha="right" if c == "B8.22" else "left", fontsize=8, color=INK2)
        pts = [("C_v2", C_V2)] + [(c, C_PROM if c in promoted else C_CAND) for c in s1["candidates"]]
        for c, col in pts:
            if c not in med.index:
                continue
            x, y = med.loc[c, "mt_s"], med.loc[c, m]
            ax.plot([q1.loc[c, "mt_s"], q3.loc[c, "mt_s"]], [y, y], color=col, lw=0.8, alpha=0.35, zorder=1)
            ax.plot([x, x], [q1.loc[c, m], q3.loc[c, m]], color=col, lw=0.8, alpha=0.35, zorder=1)
            ax.scatter([x], [y], s=60, marker="D" if c == "C_v2" else "o", color=col, edgecolor=SURFACE, linewidth=2, zorder=4)
        ax.annotate("C_v2", (med.loc["C_v2", "mt_s"], med.loc["C_v2", m]), textcoords="offset points", xytext=(7, -12),
                    fontsize=8, color=INK)
        cands = sorted([c for c in s1["candidates"] if c in med.index], key=lambda c: med.loc[c, m], reverse=True)
        bx, b_lo, b_hi = bands[m]
        for k, c in enumerate(cands):
            ty = b_hi - (b_hi - b_lo) * k / max(1, len(cands) - 1)
            ax.annotate(c, xy=(med.loc[c, "mt_s"], med.loc[c, m]), xycoords="data", xytext=(bx, ty), textcoords="axes fraction",
                        fontsize=7.5, color=INK, va="center",
                        arrowprops=dict(arrowstyle="-", color=C_CAND, lw=0.6, alpha=0.5, shrinkA=2, shrinkB=4))
        ax.set_xlabel("Movement time (s)", color=INK2)
        ax.set_ylabel(lab, color=INK2)
        ax.grid(color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=INK2, labelsize=8)
    axes[0].set_title("Speed–accuracy (S1, median, IQR whiskers)", color=INK, fontsize=10, loc="left")
    axes[1].set_title("Speed–smoothness (S1)", color=INK, fontsize=10, loc="left")
    h = [plt.Line2D([], [], color=C_FIX, lw=2, marker="o", label="fixed dampers"),
         plt.Line2D([], [], color=C_V2, ls="", marker="D", label="v2 Mode C"),
         plt.Line2D([], [], color=C_CAND, ls="", marker="o", label="v3 candidates")]
    if promoted:
        h.append(plt.Line2D([], [], color=C_PROM, ls="", marker="o", label="promoted"))
    axes[0].legend(handles=h, frameon=False, fontsize=8, labelcolor=INK2, loc="lower left")
    # forest: frontier residuals
    ax = axes[2]
    ax.set_facecolor(SURFACE)
    rows = s1["candidates"] + ["C_v2"]
    for j, (m, lab, col, off) in enumerate((("sd_lat_mm", "lateral SD", C_FIX, -0.17), ("jerk_long_rms", "long. jerk", C_CAND, 0.17))):
        for i, c in enumerate(rows):
            e = next((e for e in s1["frontier"].get(c, []) if e["metric"] == m), None)
            if not e:
                continue
            lo, hi = e["rel_ci95"]
            ax.plot([lo, hi], [i + off] * 2, color=col, lw=2, solid_capstyle="round")
            ax.scatter([e["median_rel_pct"]], [i + off], s=36, color=col if e["sig"] else SURFACE, edgecolor=col,
                       linewidth=1.6, zorder=3, label=lab if i == 0 else None)
    ax.axvline(0, color=INK2, lw=1)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(rows, fontsize=8, color=INK)
    ax.invert_yaxis()
    ax.set_xlabel("Residual vs fixed-damper curve at equal MT (%, median, 95% CI)", color=INK2, fontsize=8.5)
    ax.set_title("Frontier residual (filled = Holm p < 0.05)", color=INK, fontsize=10, loc="left")
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="lower right")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    fig.savefig(path.with_suffix(".pdf"), facecolor=SURFACE)
    plt.close(fig)


# ------------------------------------------------------------------------------------------------ report
def pct(v: float) -> str:
    return f" ({v:+.1f} %)" if np.isfinite(v) else ""


def lat_txt(v: float) -> str:
    return f"{v:.3f}" if v >= 0 else "not crossed"


def fmt_p(p: float) -> str:
    return f"{p:.2g}" if p >= 0.001 else f"{p:.1e}"


def render(res: dict) -> str:
    L = []
    s1 = res["suites"]["S1"]
    prom = res["promotion"]
    L += ["# SymbioGrip v3 innovation report", "",
          "Generated by `telemetry/scripts/analyze_innovations.py` from `telemetry/data_lab/` (written by "
          "`firmware/sim/innovation_lab.cpp`). Every number and verdict in this file is computed by that script. "
          f"Pre-registration: `docs/InnovationResearch.md` (registered under its earlier name `docs/INNOVATION_RESEARCH.md`; "
          f"renamed later, content unchanged), current SHA-256 `{res['prereg_sha256']}`; hash recorded "
          f"before the first confirmatory run (`telemetry/results/innovation/prereg.sha256`): `{res['prereg_recorded']}` "
          f"-> **{'match' if res['prereg_sha256'] == res['prereg_recorded'] else 'MISMATCH: the protocol changed after registration'}**.", "",
          f"Data: seed {s1['seed']}, N = {s1['n_seeds']} matched seeds (common random numbers) per suite. Simulation only; "
          "no hardware or human-subject data exist for any v3 candidate.", ""]
    L += ["## 1. Promotion decision (pre-registered rule, suite S1)", "",
          "| Candidate | (a) accuracy-frontier break | (b) better than C_v2 **and** B8.22, worse on no primary | (c) all checks | Promote |",
          "|---|---|---|---|---|"]
    for c, p in prom.items():
        L.append(f"| {c} | {'yes' if p['a_accuracy_frontier_break'] else 'no'} | {'yes' if p['b_better_than_C_v2_and_B8.22'] else 'no'} | "
                 f"{'pass' if p['c_all_checks'] else 'FAIL'} | **{'YES' if p['promote'] else 'no'}** |")
    np_ = [c for c, p in prom.items() if p["promote"]]
    L += ["", f"Promoted: {', '.join(np_) if np_ else 'none'}. "
          + ("`firmware/src/` is unchanged; v2 Mode C remains the shipped controller." if not np_ else ""), ""]
    mu = s1["multiplicity"]
    L += ["## 1b. Multiplicity audit and post-hoc re-analysis (deviation from the pre-registration)", "",
          f"With N = {mu['n_seeds']} seeds the smallest exact two-sided Wilcoxon p is 2/2^{mu['n_seeds']} = {mu['min_exact_p']:.5f}. "
          "Under Holm the smallest attainable adjusted p of a family of m tests is m times that:", "",
          "| Family (S1) | tests m | smallest attainable Holm p | can reach 0.05? |", "|---|---|---|---|"]
    for k, m in mu["families"].items():
        pm = mu["min_attainable_holm_p"][k]
        L.append(f"| {k} | {m} | {pm:.4f} | {'yes' if pm < ALPHA else '**no**'} |")
    L += ["", "The pre-registered families make rules (a) and (b) unattainable for every candidate, whatever the data. "
          "The pre-registered decision above is therefore uninformative about efficacy. The table below re-applies the same rule "
          "with Holm **within each candidate** (3 frontier tests; 3 primary metrics × 2 comparators = 6 tests). "
          "This correction was chosen after seeing the data. It is reported as exploratory, and it does not change "
          "the promotion decision.", "",
          "| Candidate | (a) post hoc | (b) post hoc | (c) | would promote | speed–smoothness (jerk) frontier break, post hoc |",
          "|---|---|---|---|---|---|"]
    for c, p in prom.items():
        ph = p["posthoc"]
        je = next(e for e in s1["frontier"][c] if e["metric"] == "jerk_long_rms")
        L.append(f"| {c} | {'yes' if ph['a'] else 'no'} | {'yes' if ph['b'] else 'no'} | {'pass' if p['c_all_checks'] else 'FAIL'} | "
                 f"{'YES' if ph['promote'] else 'no'} | {'**yes**' if ph['jerk_frontier_break'] else 'no'}: {je['median_rel_pct']:+.1f} % "
                 f"[{je['rel_ci95'][0]:+.1f}, {je['rel_ci95'][1]:+.1f}], per-candidate Holm p = {fmt_p(je['p_holm_cand'])} |")
    L.append("")
    L += ["## 2. Pre-registered predictions vs outcome", "", "| ID | Prediction | Outcome | Evidence |", "|---|---|---|---|"]
    for p in res["predictions"]:
        L.append(f"| {p['id']} | {p['prediction']} | {p['verdict']} | {p['detail']} |")
    L += ["", "## 3. Suite S1: medians per condition", "",
          "| Condition | MT (s) | Lateral SD (mm) | Contacts | TP ID/MT (bit/s) | TP W_e (bit/s) | Long. jerk (m/s³) | "
          "Trans. jerk (m/s³) | 6–14 Hz disp (µm) | Mean B (N·s/m) | completed / faults |", "|---|" + "---|" * 10]
    for c in s1["conditions"]:
        m, it = s1["medians"][c], s1["integrity"][c]
        L.append(f"| {c} | {m['mt_s']:.3f} | {m['sd_lat_mm']:.4f} | {m['contacts']:.1f} | {m['tp_nominal_bps']:.2f} | "
                 f"{m['tp_effective_bps']:.2f} | {m['jerk_long_rms']:.2f} | {m['jerk_trans_rms']:.2f} | {m['disp_tr_um']:.1f} | "
                 f"{m['mean_damping']:.2f} | {it['completed']}/{it['n']} / {it['faults']} |")
    L += ["", "## 4. Suite S1: paired tests vs C_v2 and B8.22", "",
          "Median paired difference (candidate − comparator), relative to the comparator median, Holm-adjusted p. "
          "Primary family = 3 metrics × candidates × 2 comparators; secondary family = 5 metrics × candidates × 2 comparators.", ""]
    for fam, ml in (("Primary", PRIMARY), ("Secondary", SECONDARY)):
        L += [f"### {fam}", "", "| Candidate | vs | " + " | ".join(lab for _, lab, *_ in ml) + " |", "|---|---|" + "---|" * len(ml)]
        for c in s1["candidates"]:
            for comp in COMPARATORS:
                cells = []
                for m, *_ in ml:
                    e = next((e for e in s1["tests"][c] if e["metric"] == m and e["comp"] == comp), None)
                    cells.append(f"{e['median_diff']:+.3g}{pct(e['rel_pct'])}, p={fmt_p(e['p_holm'])}"
                                 + (f" **{e['outcome']}**" if e["outcome"] != "n.s." else ""))
                L.append(f"| {c} | {comp} | " + " | ".join(cells) + " |")
        L.append("")
    L += ["## 5. Suite S1: residual against the fixed-damper frontier", "",
          "Per seed, the fixed-damper curve (A, B4.5, B8.22, B13, B20) is linearly interpolated at the candidate's own movement "
          "time; residual = candidate − curve. Negative = better than any fixed damper of equal speed. Holm family = 3 metrics × "
          "candidates (C_v2 reported as its own family).", "",
          "| Condition | Lateral SD residual | Contacts residual | Long. jerk residual | seeds outside fixed MT range |", "|---|---|---|---|---|"]
    for c in s1["candidates"] + ["C_v2"]:
        cells = []
        for m, _ in FRONTIER:
            e = next(e for e in s1["frontier"][c] if e["metric"] == m)
            rp = pct(e["median_rel_pct"]) if m != "contacts" else ""
            cells.append(f"{e['median']:+.3g}{rp}, p={fmt_p(e['p_holm'])} {e['outcome']}")
        out = max(e["seeds_outside_fixed_mt_range"] for e in s1["frontier"][c])
        L.append(f"| {c} | " + " | ".join(cells) + f" | {out} |")
    L += ["", "## 6. Robustness suites (candidate − comparator, median, Holm within suite)", ""]
    for sname, s in res["suites"].items():
        if sname in ("S1",):
            continue
        L += [f"### {sname}: {SUITE_DESC.get(sname, '')}", "",
              "| Condition | MT (s) | Lateral SD (mm) | Contacts | Long. jerk | ΔMT vs C_v2 | ΔMT vs B8.22 | ΔSD vs C_v2 | ΔSD vs B8.22 | Δjerk vs B8.22 | faults |",
              "|---|" + "---|" * 10]
        for c in s["conditions"]:
            m = s["medians"][c]
            row = f"| {c} | {m['mt_s']:.3f} | {m['sd_lat_mm']:.4f} | {m['contacts']:.1f} | {m['jerk_long_rms']:.2f} |"
            for metric, comp in (("mt_s", "C_v2"), ("mt_s", "B8.22"), ("sd_lat_mm", "C_v2"), ("sd_lat_mm", "B8.22"), ("jerk_long_rms", "B8.22")):
                e = next((e for e in s["tests"].get(c, []) if e["metric"] == metric and e["comp"] == comp), None)
                row += (f" {e['median_diff']:+.3g}, p={fmt_p(e['p_holm'])}{' **' + e['outcome'] + '**' if e['sig'] else ''} |" if e else " — |")
            row += f" {s['integrity'][c]['faults']} |"
            L.append(row)
        if "C_v2" in s["conditions"]:
            e = paired(res["_trials"][sname], "mt_s", "B8.22", "C_v2")
            L.append(f"\nv2 reference: C_v2 − B8.22 movement time {e['median_diff']:+.3f} s (unadjusted p = {fmt_p(e['p'])}).")
        L.append("")
    a = res["adv"]
    L += ["## 7. Passivity, quantisation and adversarial checks (suite ADV)", "",
          f"Generalised Colgate–Schenkel margin = b_phys − max_θ(−Re{{e^(−jθ)·H(e^(jθ))}}), b_phys = b_friction + K_t²/R = "
          f"{a['b_phys']:.2f} N·s/m, on {a['grid_points']} log-spaced frequencies {a['grid_hz'][0]:.1f}–{a['grid_hz'][1]:.0f} Hz "
          "(includes Nyquist). H = G − c·b_emf·L on the feed-forward path; H = G + β_max on the passivity-controller fallback path "
          "(feed-forward dropped). G is the measured rendered impedance of the frozen-difficulty force law at d = 0 and d = 1 "
          "(v2 rows: analytic G = B·L at the range ends).", "",
          "| Condition | C-S margin (N·s/m) | fallback margin | nominal work (mJ) | over-comp PC off / on (mJ) | 8 Hz rev nominal / over-comp (mJ) | "
          "quant d>0.05 (%) | 63 % envelope latency (s) | superposition err | pass |", "|---|" + "---|" * 9]
    for c, e in a["conditions"].items():
        lin = e.get("linearity_rel_err")
        L.append(f"| {c} | {e['cs_margin']:+.2f} | {e['cs_margin_fallback']:+.2f} | {e['nominal_work_j'] * 1e3:+.1f} | "
                 f"{e['over_off_work_j'] * 1e3:+.1f} / {e['over_on_work_j'] * 1e3:+.2f} | {e['rev_nom_work_j'] * 1e3:+.1f} / "
                 f"{e['rev_over_work_j'] * 1e3:+.2f} | {100 * e['quant_frac_d']:.2f} | "
                 f"{lat_txt(e['latency63_s'])} | "
                 f"{'—' if lin is None else f'{lin:.1e}'} | {'PASS' if e['pass'] else 'FAIL'} |")
    fails = {c: [k for k, v in e["criteria"].items() if not v] for c, e in a["conditions"].items() if not e["pass"]}
    if fails:
        L += ["", "Failed criteria: " + "; ".join(f"{c}: {', '.join(v)}" for c, v in fails.items()) + "."]
    L += ["", "Rendered impedance Re{G} (N·s/m), frozen difficulty:", "",
          "| Candidate | d | 0.5–2 Hz | 6–14 Hz | min Re{G} (at Hz) |", "|---|---|---|---|---|"]
    for c, e in a["conditions"].items():
        for h in e.get("rendered", []):
            L.append(f"| {c} | {h['d']} | {h['re_G_0.5_2Hz']:.2f} | {h['re_G_6_14Hz']:.2f} | {h['min_re_G']:.2f} ({h['f_min_re_G']:.1f}) |")
    cy = res["cycles"]
    L += ["", "## 8. RA4M1 timing (static bound, `telemetry/scripts/arm_cycle_budget.py`)", "",
          "| Probe | cycles (0 wait) | ×2 flash-wait bound | µs of 250 µs | library calls | pass |", "|---|---|---|---|---|---|"]
    for k, p in cy.get("probes", {}).items():
        L.append(f"| {k} | {p['cycles_zero_wait']:.0f} | {p['cycles_bound_x2']:.0f} | {p['us_bound']:.1f} | "
                 f"{', '.join(p['lib']) or '—'} | {'PASS' if p['pass'] else 'FAIL'} |")
    L += ["", f"Compiler warnings: {len(cy.get('compiler_warnings', []))}.", ""]
    L += ["## 9. Limits", "",
          "- All results come from one operator model (`firmware/sim/operator_model.h`). Its lateral error is driven by "
          "Schmidt-law y-command noise that an x-only actuator cannot reach (diagnostic D0 in the research document).",
          "- Suite 5's breathing *recovery* endpoint is not testable: the operator model has no respiration or arousal state. "
          "Only non-interference with the task was measured.",
          "- Passivity margins use a sampled-data LTI condition at frozen difficulty. Time variation is covered only by the "
          "energy observer and controller and by the closed-loop tests above, not by a proof.", ""]
    return "\n".join(L)


# ------------------------------------------------------------------------------------------------ main
def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lab", type=Path, default=REPO / "telemetry" / "data_lab")
    ap.add_argument("--out", type=Path, default=REPO / "telemetry" / "results" / "innovation")
    ap.add_argument("--figure", type=Path, default=REPO / "telemetry" / "results" / "figures" / "fig_pareto_v3.png")
    ap.add_argument("--report", type=Path, default=REPO / "InnovationReport.md")
    ap.add_argument("--adv-only", action="store_true", help="only evaluate the ADV suite (verification, no endpoints)")
    args = ap.parse_args(argv)

    adv_raw = json.loads((args.lab / "adv" / "adv.json").read_text())
    adv = adv_checks(adv_raw)
    if args.adv_only:
        for c, e in adv["conditions"].items():
            bad = [k for k, v in e["criteria"].items() if not v]
            print(f"  [{'PASS' if e['pass'] else 'FAIL'}] {c}: C-S {e['cs_margin']:+.2f}, fallback {e['cs_margin_fallback']:+.2f} N*s/m, "
                  f"nominal {e['nominal_work_j'] * 1e3:+.1f} mJ, over-comp on {e['over_on_work_j'] * 1e3:+.2f} mJ, "
                  f"rev {e['rev_nom_work_j'] * 1e3:+.1f}/{e['rev_over_work_j'] * 1e3:+.2f} mJ, quant {100 * e['quant_frac_d']:.2f} %, "
                  f"lat63 {e['latency63_s']:.3f} s {bad if bad else ''}")
            for h in e.get("rendered", []):
                print(f"        d={h['d']}: Re G 0.5-2 Hz {h['re_G_0.5_2Hz']:.2f}, 6-14 Hz {h['re_G_6_14Hz']:.2f}, "
                      f"min {h['min_re_G']:.2f} at {h['f_min_re_G']:.1f} Hz")
        return 0 if all(e["pass"] for e in adv["conditions"].values()) else 1

    res = {"suites": {}, "_trials": {}}
    for sname, sub in SUITES:
        d = args.lab / sub
        if not (d / "manifest.json").exists():
            print(f"[innov] missing suite {sname} ({d}); skipped")
            continue
        trials, man = load_suite(d)
        s = analyse_suite(trials)
        s["seed"], s["n_seeds"] = man["seed"], man["runs"]
        res["suites"][sname] = s
        res["_trials"][sname] = trials
        print(f"[innov] {sname}: {len(trials)} trials, {len(s['candidates'])} candidates")
    if "S1" not in res["suites"]:
        print("[innov] suite S1 is required")
        return 1
    cyc_path = args.out / "arm_cycles.json"
    res["cycles"] = json.loads(cyc_path.read_text()) if cyc_path.exists() else {}
    res["adv"] = adv
    res["promotion"] = promotion(res["suites"]["S1"], adv, res["cycles"])
    res["predictions"] = predictions(res)
    prereg = REPO / "docs" / "InnovationResearch.md"
    res["prereg_sha256"] = hashlib.sha256(prereg.read_bytes()).hexdigest() if prereg.exists() else "missing"
    rec = args.out / "prereg.sha256"
    res["prereg_recorded"] = rec.read_text().split()[0] if rec.exists() else "none"
    print(f"[innov] pre-registration hash {'matches' if res['prereg_sha256'] == res['prereg_recorded'] else 'DOES NOT MATCH'} the recorded value")

    promoted = [c for c, p in res["promotion"].items() if p["promote"]]
    figure(res["suites"]["S1"], res["_trials"]["S1"], args.figure, promoted)
    args.report.write_text(render(res), encoding="utf-8")
    args.out.mkdir(parents=True, exist_ok=True)
    dump = {k: v for k, v in res.items() if k != "_trials"}
    (args.out / "innovation.json").write_text(json.dumps(dump, indent=2, default=float))
    for c, p in res["promotion"].items():
        print(f"  {c:10s} a={p['a_accuracy_frontier_break']!s:5} b={p['b_better_than_C_v2_and_B8.22']!s:5} "
              f"c={p['c_all_checks']!s:5} -> {'PROMOTE' if p['promote'] else 'no'}")
    for p in res["predictions"]:
        print(f"  {p['id']}: {p['verdict']}  ({p['detail']})")
    print(f"[innov] wrote {args.report}, {args.figure}, {args.out / 'innovation.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
