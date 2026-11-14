# Control theory and its verification

Every number in this document is printed by `telemetry/scripts/verify_control_theory.py`, which also writes `telemetry/results/verification.json`. The script parses the constants straight out of `firmware/src/admittance.h` and `config.h`, and it drives the real `HapticEngine` through `libhaptic`. It exits 1 if any check fails. Current result: **19/19 PASS**.

Notation: sample period T = 2 ms (500 Hz); encoder resolution q = 1/60000 m = 16.67 µm.

## 1. Signal chain

| Stage | Design | Check |
|---|---|---|
| Velocity | backward difference, then 2nd-order Butterworth low-pass, f_c = 25 Hz (bilinear) | V1: coefficients within 4.4e-10 of `scipy.signal.butter(2, 25, fs=500)`. V2: DC gain 1.0000000, −3 dB at 25.002 Hz, max pole radius 0.8008 |
| Tremor band | acceleration = Δv_f/T, then 1st-order Butterworth high-pass at 5 Hz | V1: within 1.8e-10. V2: −3 dB at 5.001 Hz, gain at Nyquist 1.00000 |
| Tremor RMS | EMA of the squared signal, α = 1 − e^(−T/0.25 s) | V3: 0.00796809 (exact) |
| Grip | inverse FSR divider F = S·a/(1 − a), then EMA with α = 1 − e^(−T/20 ms) | V3: 0.0951626 (exact to 1e-7) |
| Damping smoother | slew limit, then critically damped 2nd-order filter, ω_n = 36 rad/s, prewarped bilinear | V1: within 3.4e-10. V2: DC 0.99999992, equivalent s-plane poles −36.016 ± 0.01j |

## 2. Quantisation noise floor (V4)

Encoder quantisation is modelled as white noise with variance q²/12 at the position input. It passes through the exact discrete chain (difference, LPF, difference, HPF), so the variance at the tremor estimator is

σ_q² = (q²/12) · Σ h[n]²,

where h is the impulse response of the whole chain. This gives **σ_q = 0.0623 m/s²**. The tremor floor `TREMOR_RMS_FLOOR` = 0.25 m/s² sits at 4.0 σ_q.

The white-noise model is optimistic for slow, smooth motion, where quantisation error is correlated. So the script also runs the real engine on pure-quantisation glides (constant speeds of 0.01–0.20 m/s, plus a 0.3 mm, 0.37 Hz wobble, floor-quantised exactly like the encoder). The worst tremor estimate is **0.0923 m/s²**, still below the floor. Quantisation alone therefore never raises the difficulty signal.

## 3. Sampled-data passivity (V5)

### Model
The carriage sees a motor force from a voltage-driven brushed DC motor:

F = (K_t/R)·V − (K_t²/R)·v.

The firmware commands V = R·i_cmd + c·K_t·v_f, with i_cmd = (−B·v_f − β·v_raw)/K_t and back-EMF compensation fraction c. Collecting terms gives

F = −(B − c·b_emf)·v_f − β·v_raw − b_emf·v,  with b_emf = K_t²/R = 14.25 N·s/m.

The term −b_emf·v is continuous-time and physical: it is the motor's own electrical damping, with no delay and no quantisation. It joins mechanical friction b_fric = 1 N·s/m to form the physical damping b_phys = 15.25 N·s/m. The sampled part is the virtual environment

H(z) = (B − c·b_emf)·(1 − z⁻¹)/T·L(z) + β·(1 − z⁻¹)/T,

which acts on position through the zero-order hold. Here L(z) is the 25 Hz low-pass.

### Condition
Colgate & Schenkel (1997) treat a sampled virtual environment H(z) coupled to a continuous mass with physical damping b. Their general condition is

b > T / (2(1 − cos ωT)) · Re{(1 − e^(−jωT)) · H(e^(jωT))}  for all ω ∈ (0, π/T].

For the plain virtual spring–damper H = K + B(1 − z⁻¹)/T this reduces to the familiar b > KT/2 + |B|. The script evaluates the general form on a 40,000-point grid.

**Sanity check:** a pure sampled damper B = 10 returns b_req = 10.0000, as the closed form predicts.

### Results
| Configuration | Worst b_req | b_phys | Verdict |
|---|---|---|---|
| Nominal: c = 1, B ∈ [4.5, 21.5], β ∈ {0, 10} | 10.00 (at B = 21.5, β = 10) | 15.25 | passive, margin 5.25 |
| Passivity-controller fallback: c = 0, B_max on v_f, β_max on v_raw | 10.00 | 15.25 | passive, margin 5.25 |
| Ideal current drive (no electrical damping), informational | 11.16 | 1.00 (friction only) | **not passive** |

Minimum electrical damping that keeps the nominal configuration passive: **K_t²/R ≥ 9.1 N·s/m** (designed 14.25). This is why the design uses voltage drive with back-EMF compensation and not ideal current control. A motor swap must preserve this margin.

Why the passivity-controller term uses raw velocity: β·v_raw contributes exactly β at the Nyquist frequency, and nothing worse anywhere else. The same term applied through the 25 Hz low-pass would be active wherever the filter's phase lag exceeds 90°.

## 4. Time-domain passivity: observer and controller

### Observer (exact for a ZOH interval)
Over one control period the terminal voltage V is constant. The work the motor does on the carriage is

E_out = ∫ F v dt = (K_t/R)·V·Δx − b_emf·∫ v² dt ≤ (K_t/R)·V·Δx − b_emf·Δx²/T.

The inequality is Cauchy–Schwarz, ∫ v² dt ≥ Δx²/T. The observer accumulates W ← W − E_out,upper (energy absorbed by the device). It is capped at +0.1 J so that a long absorbing phase cannot bank unlimited credit. Δx comes from the encoder, so the bound is exact up to one count of quantisation.

### Controller
When W < 0 and |v_raw| exceeds the deadband, the controller:

1. adds damping β = −W/(v_raw²·T), capped at 10 N·s/m, on raw velocity; and
2. drops the back-EMF feed-forward (c → 0) for that tick.

Step 2 matters. The only term that can make the device active is the velocity feed-forward. If K_t is overestimated, the feed-forward cancels more than the true back-EMF and acts as negative damping. A damping-only controller capped at β_max cannot outweigh large over-compensation. In the first test of this design, 2.5× over-compensation ≈ −21 N·s/m of negative damping. The observer correctly reported the deficit (true +35.5 mJ vs observed −35.0 mJ), but β alone could not remove it. Section 3 shows that the c = 0 fallback is passive for every B + β in range.

### Closed-loop checks (V6, V7)
20 s per check, using the real engine and plant. The compliant hand is a 50 N/m spring and 2 N·s/m damper about a 0.4 Hz target, plus 9 Hz tremor forcing and noise.

| Check | True motor work | Observer | Passivity controller active ticks |
|---|---|---|---|
| V6 nominal, mode C | −1130.33 mJ | bound −100 mJ (cap) | 0 |
| V6 nominal, mode B | −1483.69 mJ | — | 0 |
| V7 c = 2.5, controller **off** | **+64.1 mJ** (device generates energy; peak speed 0.62 m/s) | — | — |
| V7 c = 2.5, controller **on** | **−11.32 mJ** | min −2.78 mJ | 14 |

V7 is the meaningful test: it builds a configuration that is demonstrably active, then shows the controller removes the generated energy.

## 5. What is not verified

- Every check above uses the nominal motor model. Unmodelled effects on real hardware include gearbox backlash and compliance between the motor and the idler-mounted encoder, PWM-frequency current ripple, and driver dead time.
- Coulomb friction (0.2 N, present in the plant) helps passivity, and it is not credited in the frequency-domain analysis.
- None of this has been measured on hardware.
