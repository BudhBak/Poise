"""SymbioGrip benchmark figures, drawn only from evaluator.py outputs (no metric is recomputed here).

Figures (PNG + PDF) written to telemetry/results/figures/:
  fig1_paired_<dataset>   six paired panels (one thin line per seed across A/B/C, median marker);
                          the panel subtitle prints the Holm-corrected C-B p-value from evaluation.json
  fig2_controls           forest plot of the paired C-B median difference (95% bootstrap CI) for movement
                          time and lateral error SD across every dataset, incl. the null-grip and the
                          dose-matched controls
  fig3_timeseries         one seed: true grip force and applied damping over time, modes B and C

Usage:  python plot_benchmarks.py [--results telemetry/results] [--data telemetry/data_sil]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

REPO = Path(__file__).resolve().parents[2]

# Reference categorical palette, slots 1-3 (validated all-pairs, light mode); text stays in ink tokens.
MODE_COLOR = {"A": "#2a78d6", "B": "#eb6834", "C": "#1baf7a"}
MODE_LABEL = {"A": "A  no assistance", "B": "B  fixed damping", "C": "C  adaptive damping"}
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

PANELS = [
    ("mt_s", "Movement time", "s"),
    ("sd_lat_mm", "Lateral error SD", "mm"),
    ("contacts", "Wall contacts", "count"),
    ("jerk_long_rms", "Longitudinal RMS jerk", "m/s³"),
    ("jerk_trans_rms", "Transverse RMS jerk", "m/s³"),
    ("mean_damping", "Mean applied damping", "N·s/m"),
]

DATASET_LABEL = {
    "causal": "Causal grip (primary)",
    "null_grip": "Null grip (control)",
    "sensitivity_causal_dose_matched_B8.22": "Dose-matched B = 8.22",
    "sensitivity_causal_k0": "Schmidt k = 0",
    "sensitivity_causal_k0.08": "Schmidt k = 0.08",
}


def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
        "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
        "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left",
    })


def save(fig, out: Path, stem: str):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{stem}.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot] {out / stem}.png / .pdf")


def pair_stat(ev: dict, metric: str, pair: str) -> dict | None:
    for p in ev["metrics"].get(metric, {}).get("pairs", []):
        if p["pair"] == pair:
            return p
    return None


def fmt_p(p: float) -> str:
    return f"{p:.2g}" if p >= 0.001 else f"{p:.1e}"


def fig1(results: Path, name: str, out: Path):
    df = pd.read_csv(results / name / "per_trial_metrics.csv")
    ev = json.loads((results / name / "evaluation.json").read_text())
    modes = [m for m in "ABC" if m in set(df["mode"])]
    fig, axes = plt.subplots(2, 3, figsize=(10.5, 6.2), constrained_layout=True)
    rng = np.random.default_rng(0)
    for ax, (col, title, unit) in zip(axes.flat, PANELS):
        wide = df.pivot(index="run", columns="mode", values=col)[modes]
        x = np.arange(len(modes))
        jitter = rng.uniform(-0.06, 0.06, size=len(wide)) if col == "contacts" else np.zeros(len(wide))
        for j, (_, row) in enumerate(wide.iterrows()):
            ax.plot(x + jitter[j], row.values, color=INK2, alpha=0.25, lw=0.8, zorder=1)
        for i, m in enumerate(modes):
            ax.scatter(np.full(len(wide), i) + jitter, wide[m], s=22, color=MODE_COLOR[m],
                       edgecolor=SURFACE, linewidth=1.0, zorder=3)
            med = float(np.median(wide[m]))
            ax.plot([i - 0.22, i + 0.22], [med, med], color=INK, lw=2, zorder=4, solid_capstyle="round")
        ax.set_xticks(x, modes)
        ax.set_xlim(-0.5, len(modes) - 0.5)
        ax.set_ylabel(unit)
        ax.set_title(title, pad=16)
        if col == "contacts":
            ax.yaxis.get_major_locator().set_params(integer=True)
        cb = pair_stat(ev, col, "C-B")
        if cb is not None:
            ax.text(0.0, 1.01, f"C−B median {cb['median_diff']:+.3g}, Holm p = {fmt_p(cb['p_holm'])}",
                    transform=ax.transAxes, va="bottom", ha="left", fontsize=7.5, color=INK2,
                    bbox=dict(fc=SURFACE, ec="none", pad=0.5))
    handles = [plt.Line2D([], [], marker="o", ls="", color=MODE_COLOR[m], label=MODE_LABEL[m]) for m in modes]
    handles.append(plt.Line2D([], [], color=INK, lw=2, label="median"))
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, bbox_to_anchor=(0.5, -0.06))
    man = ev["info"]["manifest"]
    fig.suptitle(f"{DATASET_LABEL.get(name, name)}: SIL, {man['runs']} seeds, paired by seed "
                 f"(lines join the same seed across modes)", x=0.01, ha="left", fontsize=11, fontweight="bold")
    save(fig, out, f"fig1_paired_{name}")


def fig2(results: Path, out: Path):
    names = [n for n in DATASET_LABEL if (results / n / "evaluation.json").exists()]
    metrics = [("mt_s", "Movement time, C − B [s]"), ("sd_lat_mm", "Lateral error SD, C − B [mm]")]
    fig, axes = plt.subplots(1, 2, figsize=(10, 0.55 * len(names) + 1.4), constrained_layout=True, sharey=True)
    y = np.arange(len(names))[::-1]
    for ax, (col, title) in zip(axes, metrics):
        for yi, n in zip(y, names):
            ev = json.loads((results / n / "evaluation.json").read_text())
            p = pair_stat(ev, col, "C-B")
            lo, hi = p["ci95"]
            sig = p["p_holm"] < 0.05
            ax.plot([lo, hi], [yi, yi], color=MODE_COLOR["C"], lw=2, solid_capstyle="round")
            ax.scatter([p["median_diff"]], [yi], s=40, zorder=3, color=MODE_COLOR["C"] if sig else SURFACE,
                       edgecolor=MODE_COLOR["C"], linewidth=1.6)
            ax.text(1.0, yi, f"  p={fmt_p(p['p_holm'])}", transform=ax.get_yaxis_transform(), va="center",
                    fontsize=7.5, color=INK2)
        ax.axvline(0, color=INK2, lw=0.8)
        ax.set_title(title)
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(y, [DATASET_LABEL[n] for n in names])
    fig.suptitle("Adaptive (C) minus fixed (B): paired median difference, 95% bootstrap CI "
                 "(filled = Holm p < 0.05; negative = C lower)", x=0.01, ha="left", fontsize=10.5,
                 fontweight="bold")
    save(fig, out, "fig2_controls")


def fig3(data: Path, out: Path, run: int = 1):
    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True, constrained_layout=True)
    for m in "BC":
        f = data / "causal" / f"trial_mode_{m}_run_{run}_sil.csv"
        if not f.exists():
            continue
        d = pd.read_csv(f)
        t = d["t_ms"] / 1000.0
        axes[0].plot(t, d["grip_true_N"], color=MODE_COLOR[m], lw=1.6, label=MODE_LABEL[m])
        axes[1].plot(t, d["damping_Nspm"], color=MODE_COLOR[m], lw=2, label=MODE_LABEL[m])
        axes[1].annotate(f"{m}", (t.iloc[-1], d["damping_Nspm"].iloc[-1]), xytext=(4, 0),
                         textcoords="offset points", va="center", color=INK, fontsize=9)
    axes[0].set_ylabel("simulated grip [N]")
    axes[0].set_title("Simulated operator grip force (causal model: rises with path demand)")
    axes[1].set_ylabel("damping [N·s/m]")
    axes[1].set_xlabel("time [s]")
    axes[1].set_title("Damping applied by the firmware engine (C adapts; B is constant)")
    axes[0].legend(frameon=False, loc="upper right")
    fig.suptitle(f"Seed {run}, causal grip model", x=0.01, ha="left", fontsize=11, fontweight="bold")
    save(fig, out, "fig3_timeseries")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", type=Path, default=REPO / "telemetry" / "results")
    ap.add_argument("--data", type=Path, default=REPO / "telemetry" / "data_sil")
    args = ap.parse_args(argv)
    style()
    out = args.results / "figures"
    for name in ("causal", "null_grip", "sensitivity_causal_dose_matched_B8.22"):
        if (args.results / name / "per_trial_metrics.csv").exists():
            fig1(args.results, name, out)
    fig2(args.results, out)
    fig3(args.data, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
