#!/usr/bin/env python3
"""
Independent SRM Validation Script — MCTR 908 Group 13
======================================================
Forward-Euler re-implementation of the 6/4 SRM cascaded control drive.
Zero shared code with the MATLAB/Simulink model (src/SRM_Project.slx).

Implements:
  - Piecewise-linear inductance LUT (matches SRM_params.m)
  - Cubic TSF: f(x) = 3x² − 2x³
  - Torque-to-current inversion: i* = sqrt(2T*/dL/dθ)
  - Hysteresis current controller (bang-bang, band ±Δi)
  - Asymmetric half-bridge (AHB) converter: +Vdc / 0 / −Vdc
  - PI speed controller with clamped anti-windup

Default: TC1 Baseline — ω* = 100 rad/s, TL = 1.5 N·m step at t = 0.1 s.

Usage:
    pip install numpy matplotlib
    python srm_validation.py [--tc {1,2,3,4,5}]
    python srm_validation.py --all

Cross-validation metrics printed at the end; figures saved to ../results/.
"""

import argparse
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# ─── Machine and controller parameters ───────────────────────────────────────
RS      = 1.0          # Phase resistance [Ω]
LMIN    = 0.020        # Minimum inductance [H] (unaligned)
LMAX    = 0.150        # Maximum inductance [H] (aligned)
NPH     = 3            # Number of phases
NR      = 4            # Number of rotor poles
TAU     = np.pi / 2   # Rotor-pole pitch [rad mechanical]
PHI_K   = np.array([k * TAU / NPH for k in range(NPH)])  # Phase offsets [rad]

# Inductance profile breakpoints
TH_OFF  = np.deg2rad(38)   # Rising → aligned transition
TH_A    = np.deg2rad(43)   # Aligned → falling transition
TH_U    = np.deg2rad(81)   # Falling → unaligned transition

# Torque sharing function
THETA_OV = np.deg2rad(10)   # Overlap angle [rad]
TSF_END  = TAU / NPH        # End of full-conduction window (30°)

# Mechanical
J       = 0.002        # Rotor inertia [kg·m²]
B_FRIC  = 0.001        # Viscous damping [N·m·s/rad]

# Converter / control
VDC     = 300.0        # DC-link voltage [V]
IMAX    = 15.0         # Peak current clamp [A]
DELTA_I = 0.1          # Hysteresis half-band [A]
KP_W    = 0.5          # Speed PI proportional gain
KI_W    = 10.0         # Speed PI integral gain [s⁻¹]
T_MAX   = 5.0          # Torque anti-windup limit [N·m]

# Simulation
TS      = 1e-5         # Fixed timestep [s]
TSIM    = 0.5          # Total simulation window [s]

# ─── Test case definitions ────────────────────────────────────────────────────
TEST_CASES = {
    1: dict(w_ref=100.0, TL_val=1.5,  TL_ramp=False, w_ramp=False,
            kp=KP_W, ki=KI_W,
            label="TC1 Baseline (100 rad/s, 1.5 N·m step @ t=0.1 s)",
            tag="tc1"),
    2: dict(w_ref=200.0, TL_val=1.5,  TL_ramp=False, w_ramp=False,
            kp=KP_W, ki=KI_W,
            label="TC2 High-Speed (200 rad/s, 1.5 N·m)",
            tag="tc2"),
    3: dict(w_ref=100.0, TL_val=5.0,  TL_ramp=False, w_ramp=False,
            kp=KP_W, ki=KI_W,
            label="TC3 High-Load (100 rad/s, 5.0 N·m step @ t=0.1 s)",
            tag="tc3"),
    4: dict(w_ref=100.0, TL_val=1.5,  TL_ramp=False, w_ramp=True,
            kp=15.0,   ki=10.0,
            label="TC4 Velocity Ramp (0→100 rad/s, Kp=15)",
            tag="tc4"),
    5: dict(w_ref=100.0, TL_val=T_MAX, TL_ramp=True,  w_ramp=False,
            kp=KP_W, ki=KI_W,
            label="TC5 Torque Ramp (100 rad/s, 0→5 N·m)",
            tag="tc5"),
}

# ─── Inductance LUT ───────────────────────────────────────────────────────────
_NL    = 2000
_TH_L  = np.linspace(0, TAU, _NL, endpoint=False)
_L_L   = np.where(_TH_L < TH_OFF,
                  LMIN + (LMAX - LMIN) / TH_OFF * _TH_L,
         np.where(_TH_L < TH_A, LMAX,
         np.where(_TH_L < TH_U,
                  LMAX - (LMAX - LMIN) / (TH_U - TH_A) * (_TH_L - TH_A),
                  LMIN)))
_DL_L  = np.gradient(_L_L, _TH_L)


def lut(th_local: float):
    """Return (L, dL/dθ) for a local rotor angle in [0, TAU)."""
    idx = int(th_local % TAU / TAU * _NL) % _NL
    return _L_L[idx], _DL_L[idx]


# ─── Cubic TSF ────────────────────────────────────────────────────────────────
def cubic(x: float) -> float:
    return 3.0 * x * x - 2.0 * x * x * x


def tsf_fraction(th_k: float) -> float:
    """
    Cubic TSF fraction for phase with local angle th_k.

    Rising  [0, θ_ov)             : f = cubic(th_k / θ_ov)
    Full    [θ_ov, TSF_END)       : f = 1.0
    Falling [TSF_END, TSF_END+θ_ov): f = 1 − cubic((th_k−TSF_END)/θ_ov)
    Inactive                       : f = 0.0
    """
    if th_k < THETA_OV:
        return cubic(th_k / THETA_OV)
    elif th_k < TSF_END:
        return 1.0
    elif th_k < TSF_END + THETA_OV:
        return 1.0 - cubic((th_k - TSF_END) / THETA_OV)
    return 0.0


# ─── Simulation ───────────────────────────────────────────────────────────────
def run(tc: int = 1):
    """
    Run the SRM simulation for a given test case.

    Returns dict with arrays: t, omega, currents (N×3), Te, TL_arr.
    """
    cfg = TEST_CASES[tc]
    N   = int(TSIM / TS)
    kp  = cfg["kp"]
    ki  = cfg["ki"]

    # ── State ────────────────────────────────────────────────────────────────
    theta  = 0.0
    omega  = 0.0
    i_ph   = np.zeros(NPH)
    sw     = -np.ones(NPH, dtype=int)   # start: demagnetise
    int_w  = 0.0

    # ── Storage (decimate ×10) ────────────────────────────────────────────────
    DEC   = 10
    M     = N // DEC
    r_t   = np.empty(M)
    r_w   = np.empty(M)
    r_i   = np.empty((M, NPH))
    r_Te  = np.empty(M)
    r_TL  = np.empty(M)
    m     = 0

    t0 = time.perf_counter()

    # ── Main loop ─────────────────────────────────────────────────────────────
    for n in range(N):
        t = n * TS

        # Speed reference (ramp or step)
        if cfg["w_ramp"]:
            w_ref = min(cfg["w_ref"] * t / 0.1, cfg["w_ref"])  # 0→w_ref in 0.1 s
        else:
            w_ref = cfg["w_ref"]

        # Load torque (ramp or step)
        if cfg["TL_ramp"]:
            TL = cfg["TL_val"] * t / TSIM
        else:
            TL = cfg["TL_val"] if t >= 0.1 else 0.0

        # ── PI speed controller ───────────────────────────────────────────────
        e_w   = w_ref - omega
        T_cmd = float(np.clip(kp * e_w + ki * int_w, 0.0, T_MAX))

        # ── Per-phase inductance ──────────────────────────────────────────────
        th_loc = np.empty(NPH)
        Lk     = np.empty(NPH)
        dLk    = np.empty(NPH)
        for k in range(NPH):
            th_loc[k] = (theta - PHI_K[k]) % TAU
            Lk[k], dLk[k] = lut(th_loc[k])

        # ── TSF + T2I → current references ───────────────────────────────────
        i_ref = np.zeros(NPH)
        for k in range(NPH):
            fk = tsf_fraction(th_loc[k])
            if fk > 0.0 and dLk[k] > 1e-6:
                i_ref[k] = min(np.sqrt(2.0 * fk * T_cmd / dLk[k]), IMAX)

        # ── Hysteresis switching ──────────────────────────────────────────────
        for k in range(NPH):
            if   i_ph[k] < i_ref[k] - DELTA_I:
                sw[k] =  1   # magnetise
            elif i_ph[k] > i_ref[k] + DELTA_I:
                sw[k] = -1   # demagnetise

        # ── AHB phase voltages ────────────────────────────────────────────────
        v_ph = (sw ==  1).astype(float) * VDC \
             + (sw == -1).astype(float) * (-VDC)

        # ── Electrical + mechanical derivatives ───────────────────────────────
        didt   = (v_ph - RS * i_ph - i_ph * dLk * omega) / Lk
        Te     = float(np.sum(0.5 * i_ph ** 2 * dLk))
        domega = (Te - TL - B_FRIC * omega) / J

        # ── Store (decimated) ─────────────────────────────────────────────────
        if n % DEC == 0 and m < M:
            r_t[m]    = t
            r_w[m]    = omega
            r_i[m, :] = i_ph
            r_Te[m]   = Te
            r_TL[m]   = TL
            m += 1

        # ── Forward-Euler step ────────────────────────────────────────────────
        i_ph   = np.clip(i_ph + didt * TS, 0.0, IMAX)
        theta += omega * TS                           # use old omega
        omega  = max(omega + domega * TS, 0.0)

        # ── Integrator update (conditional anti-windup) ───────────────────────
        T_unsat = kp * e_w + ki * int_w
        if not ((T_unsat >= T_MAX and e_w >= 0) or (T_unsat <= 0.0 and e_w <= 0)):
            int_w += e_w * TS

    elapsed = time.perf_counter() - t0
    print(f"  Simulation time : {elapsed:.2f} s  ({m} samples)")

    return dict(t=r_t[:m], omega=r_w[:m], currents=r_i[:m], Te=r_Te[:m], TL=r_TL[:m])


# ─── Metrics ──────────────────────────────────────────────────────────────────
def compute_metrics(data: dict, w_ref: float) -> dict:
    t, omega, Te = data["t"], data["omega"], data["Te"]
    currents = data["currents"]

    # Steady-state window: last 20% of simulation
    ss = t > 0.8 * t[-1]
    ss_speed   = omega[ss].mean()
    ss_torque  = Te[ss].mean()
    ripple_rms = Te[ss].std()
    peak_i     = currents.max()

    # Rise time 10%–90%
    m10 = np.argmax(omega >= 0.10 * w_ref)
    m90 = np.argmax(omega >= 0.90 * w_ref)
    rise_ms = (t[m90] - t[m10]) * 1e3 if m90 > m10 else float("nan")

    return dict(
        ss_speed   = ss_speed,
        ss_torque  = ss_torque,
        ripple_rms = ripple_rms,
        peak_i     = peak_i,
        rise_ms    = rise_ms,
    )


# ─── Plotting ─────────────────────────────────────────────────────────────────
def plot(data: dict, tc: int, out_dir: Path):
    cfg   = TEST_CASES[tc]
    t     = data["t"]
    omega = data["omega"]
    i_ph  = data["currents"]
    Te    = data["Te"]
    TL    = data["TL"]

    plt.rcParams.update({
        "font.size": 8, "axes.labelsize": 8,
        "xtick.labelsize": 7, "ytick.labelsize": 7,
        "legend.fontsize": 7.5, "lines.linewidth": 0.8,
        "axes.grid": True, "grid.linestyle": ":", "grid.linewidth": 0.4,
        "grid.alpha": 0.6,
    })

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 5.2), sharex=True,
                              gridspec_kw={"hspace": 0.38})

    # Speed
    ax = axes[0]
    ax.plot(t, omega, "#1f77b4", label=r"$\omega$")
    ax.axhline(cfg["w_ref"], color="#d62728", ls="--", lw=0.85, label=r"$\omega^*$")
    ax.set_ylabel("Speed (rad/s)")
    ax.set_ylim(-5, cfg["w_ref"] * 1.25)
    ax.legend(loc="lower right", ncol=2, handlelength=1.5)
    ax.text(0.015, 0.88, "(a)", transform=ax.transAxes, fontweight="bold")

    # Phase currents
    ax = axes[1]
    for k, (cl, lb) in enumerate(
        zip(["#1f77b4", "#ff7f0e", "#2ca02c"],
            [r"$i_A$", r"$i_B$", r"$i_C$"])
    ):
        ax.plot(t, i_ph[:, k], color=cl, alpha=0.85, lw=0.55, label=lb)
    ax.set_ylabel("Phase current (A)")
    ax.set_ylim(-0.3, IMAX + 1)
    ax.legend(loc="upper right", ncol=3, handlelength=1.2)
    ax.text(0.015, 0.88, "(b)", transform=ax.transAxes, fontweight="bold")

    # Torque
    ax = axes[2]
    ax.plot(t, Te, "#9467bd", label=r"$T_e$")
    ax.plot(t, TL, "k--", lw=0.85, label=r"$T_L$")
    ax.axvline(0.1, color="gray", ls=":", lw=0.75)
    ax.set_ylabel("Torque (N·m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylim(-0.2, T_MAX + 0.5)
    ax.legend(loc="upper right", ncol=2, handlelength=1.5)
    ax.text(0.015, 0.88, "(c)", transform=ax.transAxes, fontweight="bold")

    ax.set_xlim(0, TSIM)
    fig.suptitle(cfg["label"], fontsize=8, y=0.998)

    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f"fig_{cfg['tag']}"
    fig.savefig(str(stem) + ".pdf", dpi=200, bbox_inches="tight")
    fig.savefig(str(stem) + ".png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Figure saved : {stem}.pdf / .png")


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="SRM validation script — Group 13")
    parser.add_argument("--tc", type=int, default=None, choices=[1, 2, 3, 4, 5],
                        help="Test case to run (default: 1, or use --all)")
    parser.add_argument("--all", action="store_true",
                        help="Run all 5 test cases and save all figures")
    args = parser.parse_args()

    results_dir = Path(__file__).parent.parent / "results"

    if args.all:
        print("\nRunning all 5 test cases...")
        for tc_num in range(1, 6):
            cfg = TEST_CASES[tc_num]
            print(f"\n{'='*60}")
            print(f"  SRM Validation — {cfg['label']}")
            print(f"{'='*60}")
            data = run(tc_num)
            compute_metrics(data, cfg["w_ref"])
            plot(data, tc_num, results_dir)
        print("\nAll figures saved to results/")
        return

    tc  = args.tc if args.tc is not None else 1
    cfg = TEST_CASES[tc]

    print(f"\n{'='*60}")
    print(f"  SRM Validation — {cfg['label']}")
    print(f"{'='*60}")

    data    = run(tc)
    metrics = compute_metrics(data, cfg["w_ref"])

    print(f"\n  {'Metric':<30} {'Value':>12}")
    print(f"  {'-'*44}")
    print(f"  {'SS speed (rad/s)':<30} {metrics['ss_speed']:>12.4f}")
    print(f"  {'Rise time 10-90% (ms)':<30} {metrics['rise_ms']:>12.1f}")
    print(f"  {'Mean EM torque SS (N·m)':<30} {metrics['ss_torque']:>12.4f}")
    print(f"  {'Torque ripple RMS (N·m)':<30} {metrics['ripple_rms']:>12.4f}")
    print(f"  {'Peak phase current (A)':<30} {metrics['peak_i']:>12.3f}")

    if tc == 1:
        print(f"\n  Cross-validation vs Simulink (TC1):")
        ref = dict(ss_speed=99.94, rise_ms=29.4, ss_torque=1.601,
                   ripple_rms=0.337, peak_i=7.21)
        checks = [
            ("SS speed",    abs(metrics["ss_speed"]  - ref["ss_speed"])  / ref["ss_speed"]  * 100, 1.0,  "%"),
            ("Rise time",   abs(metrics["rise_ms"]   - ref["rise_ms"]),                             15.0, "ms"),
            ("Mean torque", abs(metrics["ss_torque"] - ref["ss_torque"]) / ref["ss_torque"] * 100, 2.0,  "%"),
            ("Peak i",      abs(metrics["peak_i"]    - ref["peak_i"])    / ref["peak_i"]    * 100, 5.0,  "%"),
        ]
        all_pass = True
        for name, delta, tol, unit in checks:
            status = "PASS" if delta <= tol else "FAIL"
            if status == "FAIL":
                all_pass = False
            print(f"    {name:<16} Δ = {delta:6.2f} {unit:<3}  tol ≤ {tol} {unit}  [{status}]")
        print(f"\n  Overall: {'ALL PASS ✓' if all_pass else 'SOME FAIL ✗'}")

    # Save figure relative to this script's location (defined at top of main)
    plot(data, tc, results_dir)


if __name__ == "__main__":
    main()
