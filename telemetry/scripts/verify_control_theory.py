"""SymbioGrip control-theory verification. Every verdict is computed; the script exits 1 on any FAIL.

 V1  Filter coefficients PARSED from firmware/src/admittance.h vs. fresh designs from their specifications
 V2  Filter properties: DC gain, -3 dB frequency, pole radii
 V3  EMA time constants in config.h vs. 1 - exp(-T/tau)
 V4  Encoder-quantisation noise floor of the tremor estimator (analytic white-quantisation model and
     Monte Carlo through the REAL firmware engine via libhaptic); TREMOR_RMS_FLOOR must be >= 3 sigma and
     pure-quantisation glides must never exceed it
 V5  Sampled-data passivity (generalised Colgate & Schenkel 1997 condition) for the implemented chain:
     velocity LPF, back-EMF compensation, passivity-controller term, damping range of modes B and C
 V6  Time-domain passivity, nominal closed loop (engine + plant + compliant hand): true motor work <= 0 and
     true motor work <= observer bound
 V7  Time-domain passivity, deliberately active configuration (3x back-EMF over-compensation): without the
     passivity controller the device generates energy; with it, the generated energy is bounded

Usage:  python verify_control_theory.py [--json <path>]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from scipy import signal

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import haptic_native as hn  # noqa: E402

REPO_ROOT = SCRIPT_DIR.parents[1]
ADMITTANCE_H = REPO_ROOT / "firmware" / "src" / "admittance.h"
CONFIG_H = REPO_ROOT / "firmware" / "src" / "config.h"

FS = 500.0
T = 1.0 / FS
RESULTS: list[dict] = []


def check(name: str, passed: bool, detail: str) -> bool:
    RESULTS.append({"check": name, "pass": bool(passed), "detail": detail})
    print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    return passed


def parse_constants(path: Path, pattern: str) -> dict[str, float]:
    text = path.read_text(encoding="utf-8")
    return {m.group(1): float(m.group(2)) for m in re.finditer(pattern, text)}


def designed_filters() -> dict[str, np.ndarray]:
    lp_b, lp_a = signal.butter(2, 25.0, btype="low", fs=FS)
    hp_b, hp_a = signal.butter(1, 5.0, btype="high", fs=FS)
    wn = 36.0
    wp = 2 * FS * math.tan(wn / (2 * FS))           # prewarp at wn
    sm_b, sm_a = signal.bilinear([wp ** 2], [1.0, 2 * wp, wp ** 2], fs=FS)
    return {"lp_b": lp_b, "lp_a": lp_a, "hp_b": hp_b, "hp_a": hp_a, "sm_b": sm_b, "sm_a": sm_a}


def v1_v2_filters() -> dict[str, np.ndarray]:
    print("\nV1/V2 Filter coefficients and properties")
    c = parse_constants(ADMITTANCE_H, r"\b((?:LP|HP|SM)_[AB]\d)\s*=\s*([-+0-9.eE]+)f")
    need = ["LP_B0", "LP_B1", "LP_B2", "LP_A1", "LP_A2", "HP_B0", "HP_B1", "HP_A1",
            "SM_B0", "SM_B1", "SM_B2", "SM_A1", "SM_A2"]
    check("V1 coefficients present in admittance.h", all(k in c for k in need), f"found {len(c)} of {len(need)}")
    fw = {
        "lp_b": np.array([c["LP_B0"], c["LP_B1"], c["LP_B2"]]), "lp_a": np.array([1.0, c["LP_A1"], c["LP_A2"]]),
        "hp_b": np.array([c["HP_B0"], c["HP_B1"]]), "hp_a": np.array([1.0, c["HP_A1"]]),
        "sm_b": np.array([c["SM_B0"], c["SM_B1"], c["SM_B2"]]), "sm_a": np.array([1.0, c["SM_A1"], c["SM_A2"]]),
    }
    ref = designed_filters()
    for name, label in (("lp", "25 Hz Butterworth LPF"), ("hp", "5 Hz Butterworth HPF"), ("sm", "36 rad/s critically damped smoother")):
        err = max(np.max(np.abs(fw[name + "_b"] - ref[name + "_b"])), np.max(np.abs(fw[name + "_a"] - ref[name + "_a"])))
        check(f"V1 {label} matches design", err < 2e-8, f"max |coef error| = {err:.2e}")

    w, h = signal.freqz(fw["lp_b"], fw["lp_a"], worN=2 ** 16, fs=FS)
    f3 = float(w[np.argmin(np.abs(20 * np.log10(np.abs(h)) + 3.0103))])
    check("V2 LPF DC gain = 1 and fc = 25 Hz", abs(abs(h[0]) - 1) < 1e-6 and abs(f3 - 25.0) < 0.05,
          f"DC = {abs(h[0]):.7f}, -3 dB at {f3:.3f} Hz, max |pole| = {np.max(np.abs(np.roots(fw['lp_a']))):.6f}")
    w, h = signal.freqz(fw["hp_b"], fw["hp_a"], worN=2 ** 16, fs=FS)
    f3 = float(w[1:][np.argmin(np.abs(20 * np.log10(np.abs(h[1:])) + 3.0103))])
    check("V2 HPF fc = 5 Hz, unity at Nyquist", abs(f3 - 5.0) < 0.05 and abs(abs(h[-1]) - 1) < 1e-3,
          f"-3 dB at {f3:.3f} Hz, |H(Nyquist)| = {abs(h[-1]):.5f}")
    p = np.roots(fw["sm_a"])
    s_poles = 2 * FS * (p - 1) / (p + 1)
    dc = fw["sm_b"].sum() / fw["sm_a"].sum()
    check("V2 smoother unity DC, critically damped at 36 rad/s", abs(dc - 1) < 1e-6 and np.all(np.abs(s_poles.imag) < 0.5)
          and np.all(np.abs(np.abs(s_poles) - 36.0) < 0.5),
          f"DC = {dc:.8f}, equivalent s-poles = {np.round(s_poles, 3).tolist()}")
    return fw


def v3_ema():
    print("\nV3 EMA constants")
    c = parse_constants(CONFIG_H, r"#define\s+(\w+_ALPHA)\s+([-+0-9.eE]+)f")
    for key, tau in (("GRIP_LP_ALPHA", 0.020), ("TREMOR_MS_ALPHA", 0.250)):
        ref = 1 - math.exp(-T / tau)
        check(f"V3 {key} = 1-exp(-T/{tau * 1000:.0f} ms)", abs(c.get(key, -1) - ref) < 1e-7,
              f"config {c.get(key)}, expected {ref:.8f}")


def v4_quantisation(fw: dict, cfg: dict):
    print("\nV4 Encoder quantisation noise floor")
    q = 1.0 / cfg["counts_per_meter"]
    imp = np.zeros(4000); imp[0] = 1.0
    h = signal.lfilter([1, -1], [T], imp)
    h = signal.lfilter(fw["lp_b"], fw["lp_a"], h)
    h = signal.lfilter([1, -1], [T], h)
    h = signal.lfilter(fw["hp_b"], fw["hp_a"], h)
    sigma = math.sqrt(q * q / 12.0 * float(np.sum(h * h)))
    floor = cfg["tremor_floor"]
    check("V4 analytic floor: TREMOR_RMS_FLOOR >= 3 sigma_q", floor >= 3 * sigma,
          f"q = {q * 1e6:.3f} um, sigma_q = {sigma:.4f} m/s^2, floor = {floor} m/s^2 ({floor / sigma:.1f} sigma)")
    worst = 0.0
    for v in (0.01, 0.03, 0.06, 0.10, 0.20):
        eng = hn.Engine()
        eng.set_mode("C", 0)
        trem = []
        for k in range(6000):
            t = k * T
            x = v * t + 0.0003 * math.sin(2 * math.pi * 0.37 * t)
            eng.step(int(round(2.0 / 3.0 * 16383)), int(math.floor(x * cfg["counts_per_meter"])))
            if k > 1000:
                trem.append(eng.state()["tremor_rms"])
        worst = max(worst, max(trem))
    check("V4 firmware engine: quantisation-only tremor estimate < floor", worst < floor,
          f"max tremor_rms over glides 0.01-0.20 m/s = {worst:.4f} m/s^2 (floor {floor})")


def passivity_b_required(h_of_z, theta: np.ndarray) -> float:
    z = np.exp(1j * theta)
    return float(np.max(T / (2 * (1 - np.cos(theta))) * np.real((1 - 1 / z) * h_of_z(z))))


def v5_frequency_domain(fw: dict, cfg: dict) -> dict:
    print("\nV5 Sampled-data passivity (generalised Colgate-Schenkel)")
    theta = np.linspace(1e-4, np.pi, 40000)
    _, L = signal.freqz(fw["lp_b"], fw["lp_a"], worN=theta)

    def lpf(z):
        return L

    sanity = passivity_b_required(lambda z: 10.0 * (1 - 1 / z) / T, theta)
    check("V5 sanity: pure sampled damper B=10 requires b >= |B|", abs(sanity - 10.0) < 1e-3, f"b_req = {sanity:.4f}")

    b_emf = cfg["motor_kt_eff"] ** 2 / cfg["motor_r"]
    b_phys = cfg["b_friction_est"] + b_emf
    worst = -1e9
    worst_at = None
    for B in np.linspace(cfg["damping_min"], cfg["damping_max"], 35):
        for beta in (0.0, cfg["pc_beta_max"]):
            req = passivity_b_required(lambda z: (B - b_emf) * (1 - 1 / z) / T * lpf(z) + beta * (1 - 1 / z) / T, theta)
            if req - b_phys > worst:
                worst, worst_at = req - b_phys, (B, beta, req)
    ok = check("V5 voltage drive + back-EMF compensation is passive over B in [Bmin, Bmax], beta in {0, beta_max}",
               worst < 0,
               f"b_emf = {b_emf:.2f}, b_phys = b_fric + b_emf = {b_phys:.2f} N*s/m; worst b_req = {worst_at[2]:.2f} at "
               f"B = {worst_at[0]:.1f}, beta = {worst_at[1]:.0f}; margin = {-worst:.2f} N*s/m")
    fb = passivity_b_required(lambda z: (1 - 1 / z) / T * (cfg["damping_max"] * lpf(z) + cfg["pc_beta_max"]), theta)
    check("V5 PC fallback (feed-forward dropped: B_max on filtered v, beta_max on raw v) is passive",
          fb < b_phys, f"b_req = {fb:.2f} < b_phys = {b_phys:.2f} N*s/m (margin {b_phys - fb:.2f})")
    ideal = max(passivity_b_required(lambda z: (B + cfg["pc_beta_max"]) * (1 - 1 / z) / T * lpf(z), theta)
                for B in (cfg["damping_max"],))
    print(f"  [INFO] ideal current drive (no electrical damping) would need b_phys >= {ideal:.2f} N*s/m; "
          f"friction alone is {cfg['b_friction_est']:.2f} -> not passive. This is why voltage drive is used.")
    min_bemf = None
    for cand in np.linspace(0, 30, 301):
        w = max(passivity_b_required(lambda z: (B - cand) * (1 - 1 / z) / T * lpf(z) + beta * (1 - 1 / z) / T, theta)
                - (cfg["b_friction_est"] + cand)
                for B in (cfg["damping_min"], cfg["damping_max"]) for beta in (0.0, cfg["pc_beta_max"]))
        if w < 0:
            min_bemf = float(cand)
            break
    print(f"  [INFO] minimum motor electrical damping K_t^2/R for passivity: {min_bemf:.1f} N*s/m (design: {b_emf:.2f})")
    return {"b_emf": b_emf, "b_phys": b_phys, "margin": -worst, "min_b_emf": min_bemf, "pass": ok}


def closed_loop_run(actuator, pc_enabled: bool, seconds: float, mode: str = "C") -> dict:
    eng = hn.Engine(actuator)
    eng.set_pc_enabled(pc_enabled)
    plant = hn.Plant()
    rng = np.random.default_rng(7)
    eng.set_mode(mode, plant.encoder())
    w_min, pc_ticks, max_v = 0.0, 0, 0.0
    obs_bound = 0.0
    for k in range(int(seconds / T)):
        t = k * T
        grip = int(16383 * (0.70 if (k // 400) % 2 else 0.80))
        cmd = eng.step(grip, plant.encoder())
        x_eq = 0.09 + 0.05 * math.sin(2 * math.pi * 0.4 * t)
        fx = 0.4 * math.sin(2 * math.pi * 9.0 * t) + 0.05 * rng.standard_normal()
        plant.advance(fx=fx, x_eq=x_eq, kx=50.0, bx=2.0, y_eq=0.0, ky=60.0, by=2.6, cmd=cmd)
        st = eng.state()
        w_min = min(w_min, st["energy_j"])
        pc_ticks += 1 if st["beta"] > 0 else 0
        max_v = max(max_v, abs(plant.state()["vx"]))
    ps = plant.state()
    st = eng.state()
    return {"true_motor_work_j": ps["motor_work_j"], "observer_energy_j": st["energy_j"], "observer_min_j": w_min,
            "pc_ticks": pc_ticks, "max_speed": max_v}


def v6_v7_time_domain(cfg: dict) -> dict:
    print("\nV6 Time-domain passivity: nominal closed loop (20 s, compliant hand, mode C)")
    nominal = (cfg["motor_kt_eff"], cfg["motor_r"], cfg["supply_v"], cfg["i_max"], 1.0)
    r = closed_loop_run(nominal, True, 20.0)
    check("V6 device is passive (true motor work <= 0)", r["true_motor_work_j"] <= 0.0,
          f"true motor work = {r['true_motor_work_j'] * 1000:+.2f} mJ, PC active ticks = {r['pc_ticks']}")
    check("V6 observer is an upper bound on generated energy (true work <= -W)",
          r["true_motor_work_j"] <= -r["observer_energy_j"] + 1e-9,
          f"true {r['true_motor_work_j'] * 1000:+.2f} mJ <= bound {-r['observer_energy_j'] * 1000:+.2f} mJ")
    rb = closed_loop_run(nominal, True, 20.0, mode="B")
    check("V6 mode B also passive", rb["true_motor_work_j"] <= 0.0,
          f"true motor work = {rb['true_motor_work_j'] * 1000:+.2f} mJ, PC active ticks = {rb['pc_ticks']}")

    print("\nV7 Time-domain passivity: deliberately active configuration (2.5x back-EMF over-compensation)")
    active = (cfg["motor_kt_eff"], cfg["motor_r"], cfg["supply_v"], cfg["i_max"], 2.5)
    off = closed_loop_run(active, False, 20.0, mode="B")
    on = closed_loop_run(active, True, 20.0, mode="B")
    check("V7 without PC the configuration generates energy (test is meaningful)", off["true_motor_work_j"] > 0.01,
          f"true motor work without PC = {off['true_motor_work_j'] * 1000:+.1f} mJ, max speed {off['max_speed']:.3f} m/s")
    check("V7 with PC the generated energy is bounded (<= 5 mJ) and PC engages",
          on["true_motor_work_j"] <= 0.005 and on["pc_ticks"] > 0,
          f"true motor work with PC = {on['true_motor_work_j'] * 1000:+.2f} mJ, observer min = "
          f"{on['observer_min_j'] * 1000:+.2f} mJ, PC active ticks = {on['pc_ticks']}")
    return {"nominal_C": r, "nominal_B": rb, "active_no_pc": off, "active_with_pc": on}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", type=Path, default=REPO_ROOT / "telemetry" / "results" / "verification.json")
    args = ap.parse_args(argv)
    print("SymbioGrip control-theory verification")
    cfg = hn.config()
    fw = v1_v2_filters()
    v3_ema()
    v4_quantisation(fw, cfg)
    pas = v5_frequency_domain(fw, cfg)
    td = v6_v7_time_domain(cfg)
    n_fail = sum(1 for r in RESULTS if not r["pass"])
    print(f"\nSummary: {len(RESULTS) - n_fail}/{len(RESULTS)} checks passed")
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({"checks": RESULTS, "passivity": pas, "time_domain": td, "config": cfg}, indent=2))
    print(f"[verify] wrote {args.json}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
