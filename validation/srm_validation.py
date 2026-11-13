"""Independent SRM Validation Script -- MCTR 908 Group 13.

Forward-Euler re-implementation of the 6/4 SRM cascaded control drive with zero
shared code with the MATLAB/Simulink model (src/SRM_Project.slx).

Implements:
  - Piecewise-linear inductance LUT (matches SRM_params.m)
  - Cubic TSF: f(x) = 3x^2 - 2x^3
  - Torque-to-current inversion: i* = sqrt(2 * T* / (dL/dtheta))
  - Hysteresis current controller (bang-bang, band +/- delta_i)
  - Asymmetric half-bridge (AHB) converter: +Vdc / 0 / -Vdc
  - PI speed controller with conditional anti-windup

Usage:
    uv run python validation/srm_validation.py [--tc {1,2,3,4,5}]
    uv run python validation/srm_validation.py --all
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ─── Machine and controller parameters ───────────────────────────────────────
RS = 1.0  # Phase resistance [Ohm]
LMIN = 0.020  # Minimum inductance [H] (unaligned)
LMAX = 0.150  # Maximum inductance [H] (aligned)
NPH = 3  # Number of phases
NR = 4  # Number of rotor poles
TAU = np.pi / 2  # Rotor-pole pitch [rad mechanical]
PHI_K = np.array([k * TAU / NPH for k in range(NPH)])  # Phase offsets [rad]

# Inductance profile breakpoints
TH_OFF = float(np.deg2rad(38))  # Rising -> aligned transition
TH_A = float(np.deg2rad(43))  # Aligned -> falling transition
TH_U = float(np.deg2rad(81))  # Falling -> unaligned transition

# Torque sharing function
THETA_OV = float(np.deg2rad(10))  # Overlap angle [rad]
TSF_END = TAU / NPH  # End of full-conduction window (30 deg)

# Mechanical
J = 0.002  # Rotor inertia [kg*m^2]
B_FRIC = 0.001  # Viscous damping [N*m*s/rad]

# Converter / control
VDC = 300.0  # DC-link voltage [V]
IMAX = 15.0  # Peak current clamp [A]
DELTA_I = 0.1  # Hysteresis half-band [A]
KP_W = 0.5  # Speed PI proportional gain
KI_W = 10.0  # Speed PI integral gain [s^-1]
T_MAX = 5.0  # Torque anti-windup limit [N*m]

# Simulation & reference timing
TS = 1e-5  # Fixed timestep [s]
TSIM = 0.5  # Total simulation window [s]
LOAD_STEP_TIME = 0.1  # Load torque step instant [s]
SPEED_RAMP_TIME = 0.1  # TC4 speed ramp duration [s]
DECIMATION = 10  # Storage decimation factor
MIN_DL_THRESHOLD = 1e-6  # Minimum dL/dtheta for T2I inversion [H/rad]

SIMULINK_TC1_REF: dict[str, float] = {
    "ss_speed": 99.94,
    "rise_ms": 29.4,
    "ss_torque": 1.601,
    "ripple_rms": 0.337,
    "peak_i": 7.21,
}

# ─── Test case definitions ───────────────────────────────────────────────────
TEST_CASES: dict[int, dict[str, Any]] = {
    1: {
        "w_ref": 100.0,
        "TL_val": 1.5,
        "TL_ramp": False,
        "w_ramp": False,
        "kp": KP_W,
        "ki": KI_W,
        "label": "TC1 Baseline (100 rad/s, 1.5 N·m step @ t=0.1 s)",
        "tag": "tc1",
    },
    2: {
        "w_ref": 200.0,
        "TL_val": 1.5,
        "TL_ramp": False,
        "w_ramp": False,
        "kp": KP_W,
        "ki": KI_W,
        "label": "TC2 High-Speed (200 rad/s, 1.5 N·m)",
        "tag": "tc2",
    },
    3: {
        "w_ref": 100.0,
        "TL_val": 5.0,
        "TL_ramp": False,
        "w_ramp": False,
        "kp": KP_W,
        "ki": KI_W,
        "label": "TC3 High-Load (100 rad/s, 5.0 N·m step @ t=0.1 s)",
        "tag": "tc3",
    },
    4: {
        "w_ref": 100.0,
        "TL_val": 1.5,
        "TL_ramp": False,
        "w_ramp": True,
        "kp": 15.0,
        "ki": 10.0,
        "label": "TC4 Velocity Ramp (0→100 rad/s, Kp=15)",
        "tag": "tc4",
    },
    5: {
        "w_ref": 100.0,
        "TL_val": T_MAX,
        "TL_ramp": True,
        "w_ramp": False,
        "kp": KP_W,
        "ki": KI_W,
        "label": "TC5 Torque Ramp (100 rad/s, 0→5 N·m)",
        "tag": "tc5",
    },
}

# ─── Inductance LUT ──────────────────────────────────────────────────────────
_NL = 2000
_TH_L = np.linspace(0, TAU, _NL, endpoint=False)
_L_L = np.where(
    _TH_L < TH_OFF,
    LMIN + (LMAX - LMIN) / TH_OFF * _TH_L,
    np.where(
        _TH_L < TH_A,
        LMAX,
        np.where(
            _TH_L < TH_U,
            LMAX - (LMAX - LMIN) / (TH_U - TH_A) * (_TH_L - TH_A),
            LMIN,
        ),
    ),
)
_DL_L = np.gradient(_L_L, _TH_L)


def lut(th_local: float) -> tuple[float, float]:
    """Return (L, dL/dtheta) for a local rotor angle in [0, TAU)."""
    idx = int(th_local % TAU / TAU * _NL) % _NL
    return float(_L_L[idx]), float(_DL_L[idx])


# ─── Cubic TSF ───────────────────────────────────────────────────────────────
def cubic(x: float) -> float:
    """Evaluate the Hermite cubic blend 3x^2 - 2x^3 on [0, 1]."""
    return 3.0 * x * x - 2.0 * x * x * x


def tsf_fraction(th_k: float) -> float:
    """Compute cubic TSF fraction for a phase at local rotor angle th_k.

    Rising  [0, theta_ov)                 : f = cubic(th_k / theta_ov)
    Full    [theta_ov, TSF_END)           : f = 1.0
    Falling [TSF_END, TSF_END + theta_ov) : f = 1 - cubic((th_k - TSF_END) / theta_ov)
    Inactive                              : f = 0.0
    """
    if th_k < THETA_OV:
        return cubic(th_k / THETA_OV)
    if th_k < TSF_END:
        return 1.0
    if th_k < TSF_END + THETA_OV:
        return 1.0 - cubic((th_k - TSF_END) / THETA_OV)
    return 0.0


# ─── Simulation ──────────────────────────────────────────────────────────────
def _reference_inputs(cfg: dict[str, Any], t: float) -> tuple[float, float]:
    """Return (w_ref, load_torque) at simulation time t."""
    if cfg["w_ramp"]:
        w_ref = min(cfg["w_ref"] * t / SPEED_RAMP_TIME, cfg["w_ref"])
    else:
        w_ref = float(cfg["w_ref"])

    if cfg["TL_ramp"]:
        load_torque = float(cfg["TL_val"]) * t / TSIM
    else:
        load_torque = float(cfg["TL_val"]) if t >= LOAD_STEP_TIME else 0.0
    return w_ref, load_torque


def run(tc: int = 1) -> dict[str, np.ndarray]:
    """Run the SRM simulation for a given test case (1..5).

    Returns a dictionary containing time-series arrays:
    ``t``, ``w_ref``, ``omega``, ``currents`` (M x 3), ``Te``, and ``TL``.
    """
    if tc not in TEST_CASES:
        raise ValueError(f"Unknown test case {tc}; expected one of {sorted(TEST_CASES)}")

    cfg = TEST_CASES[tc]
    n_steps = round(TSIM / TS)
    kp = float(cfg["kp"])
    ki = float(cfg["ki"])

    theta = 0.0
    omega = 0.0
    i_ph = np.zeros(NPH)
    sw = -np.ones(NPH, dtype=int)  # Start demagnetized
    int_w = 0.0

    m_samples = n_steps // DECIMATION
    r_t = np.empty(m_samples)
    r_w_ref = np.empty(m_samples)
    r_w = np.empty(m_samples)
    r_i = np.empty((m_samples, NPH))
    r_te = np.empty(m_samples)
    r_tl = np.empty(m_samples)
    m = 0

    t0 = time.perf_counter()

    for n in range(n_steps):
        t = n * TS
        w_ref, load_torque = _reference_inputs(cfg, t)

        e_w = w_ref - omega
        t_cmd = float(np.clip(kp * e_w + ki * int_w, 0.0, T_MAX))

        th_loc = np.empty(NPH)
        l_k = np.empty(NPH)
        dl_k = np.empty(NPH)
        for k in range(NPH):
            th_loc[k] = (theta - PHI_K[k]) % TAU
            l_k[k], dl_k[k] = lut(float(th_loc[k]))

        i_ref = np.zeros(NPH)
        for k in range(NPH):
            fk = tsf_fraction(float(th_loc[k]))
            if fk > 0.0 and dl_k[k] > MIN_DL_THRESHOLD:
                i_ref[k] = min(np.sqrt(2.0 * fk * t_cmd / dl_k[k]), IMAX)

        for k in range(NPH):
            if i_ph[k] < i_ref[k] - DELTA_I:
                sw[k] = 1
            elif i_ph[k] > i_ref[k] + DELTA_I:
                sw[k] = -1

        v_ph = (sw == 1).astype(float) * VDC + (sw == -1).astype(float) * (-VDC)
        didt = (v_ph - RS * i_ph - i_ph * dl_k * omega) / l_k
        te = float(np.sum(0.5 * i_ph**2 * dl_k))
        domega = (te - load_torque - B_FRIC * omega) / J

        if n % DECIMATION == 0 and m < m_samples:
            r_t[m] = t
            r_w_ref[m] = w_ref
            r_w[m] = omega
            r_i[m, :] = i_ph
            r_te[m] = te
            r_tl[m] = load_torque
            m += 1

        # Forward-Euler state integration (theta uses pre-step omega)
        i_ph = np.clip(i_ph + didt * TS, 0.0, IMAX)
        theta += omega * TS
        omega = max(omega + domega * TS, 0.0)

        # Conditional anti-windup clamping
        t_unsat = kp * e_w + ki * int_w
        if not ((t_unsat >= T_MAX and e_w >= 0.0) or (t_unsat <= 0.0 and e_w <= 0.0)):
            int_w += e_w * TS

    elapsed = time.perf_counter() - t0
    print(f"  Simulation time : {elapsed:.2f} s  ({m} samples)")

    return {
        "t": r_t[:m],
        "w_ref": r_w_ref[:m],
        "omega": r_w[:m],
        "currents": r_i[:m],
        "Te": r_te[:m],
        "TL": r_tl[:m],
    }


# ─── Metrics ─────────────────────────────────────────────────────────────────
def compute_metrics(data: dict[str, np.ndarray], w_ref: float) -> dict[str, float]:
    """Compute steady-state and transient metrics from simulation output."""
    t, omega, te = data["t"], data["omega"], data["Te"]
    currents = data["currents"]

    ss = t > 0.8 * t[-1]
    ss_speed = float(omega[ss].mean())
    ss_torque = float(te[ss].mean())
    ripple_rms = float(te[ss].std())
    peak_i = float(currents.max())

    m10 = int(np.argmax(omega >= 0.10 * w_ref))
    m90 = int(np.argmax(omega >= 0.90 * w_ref))
    rise_ms = float((t[m90] - t[m10]) * 1e3) if m90 > m10 else float("nan")

    return {
        "ss_speed": ss_speed,
        "ss_torque": ss_torque,
        "ripple_rms": ripple_rms,
        "peak_i": peak_i,
        "rise_ms": rise_ms,
    }


# ─── Plotting ────────────────────────────────────────────────────────────────
def _apply_plot_style() -> None:
    """Apply consistent Matplotlib typography and grid styling."""
    plt.rcParams.update(
        {
            "font.size": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7.5,
            "lines.linewidth": 0.8,
            "axes.grid": True,
            "grid.linestyle": ":",
            "grid.linewidth": 0.4,
            "grid.alpha": 0.6,
        }
    )


def _save_figure(fig: plt.Figure, out_dir: Path, stem_name: str) -> Path:
    """Save figure as both PDF and PNG and close it."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / stem_name
    fig.savefig(f"{stem}.pdf", dpi=200, bbox_inches="tight")
    fig.savefig(f"{stem}.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Figure saved : {stem}.pdf / .png")
    return stem


def plot(data: dict[str, np.ndarray], tc: int, out_dir: Path) -> None:
    """Plot speed, phase currents, and electromagnetic torque for one test case."""
    cfg = TEST_CASES[tc]
    t = data["t"]
    w_ref = data["w_ref"]
    omega = data["omega"]
    i_ph = data["currents"]
    te = data["Te"]
    tl = data["TL"]

    _apply_plot_style()
    fig, axes = plt.subplots(3, 1, figsize=(3.5, 5.2), sharex=True, gridspec_kw={"hspace": 0.38})

    ax = axes[0]
    ax.plot(t, omega, "#1f77b4", label=r"$\omega$")
    ax.plot(t, w_ref, color="#d62728", ls="--", lw=0.85, label=r"$\omega^*$")
    ax.set_ylabel("Speed (rad/s)")
    ax.set_ylim(-5, cfg["w_ref"] * 1.25)
    ax.legend(loc="lower right", ncol=2, handlelength=1.5)
    ax.text(0.015, 0.88, "(a)", transform=ax.transAxes, fontweight="bold")

    ax = axes[1]
    for k, (color, label) in enumerate(
        zip(["#1f77b4", "#ff7f0e", "#2ca02c"], [r"$i_A$", r"$i_B$", r"$i_C$"], strict=True)
    ):
        ax.plot(t, i_ph[:, k], color=color, alpha=0.85, lw=0.55, label=label)
    ax.set_ylabel("Phase current (A)")
    ax.set_ylim(-0.3, IMAX + 1)
    ax.legend(loc="upper right", ncol=3, handlelength=1.2)
    ax.text(0.015, 0.88, "(b)", transform=ax.transAxes, fontweight="bold")

    ax = axes[2]
    ax.plot(t, te, "#9467bd", label=r"$T_e$")
    ax.plot(t, tl, "k--", lw=0.85, label=r"$T_L$")
    ax.axvline(LOAD_STEP_TIME, color="gray", ls=":", lw=0.75)
    ax.set_ylabel("Torque (N·m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylim(-0.2, T_MAX + 0.5)
    ax.legend(loc="upper right", ncol=2, handlelength=1.5)
    ax.text(0.015, 0.88, "(c)", transform=ax.transAxes, fontweight="bold")

    ax.set_xlim(0, TSIM)
    fig.suptitle(cfg["label"], fontsize=8, y=0.998)
    _save_figure(fig, out_dir, f"fig_{cfg['tag']}")


def plot_operating_envelope(all_data: dict[int, dict[str, np.ndarray]], out_dir: Path) -> None:
    """Generate the 4-panel operating envelope composite figure for TC2-TC5."""
    _apply_plot_style()
    fig, axes = plt.subplots(2, 2, figsize=(8.2, 6.0), gridspec_kw={"hspace": 0.34, "wspace": 0.24})

    # Panel (a): TC2 High-speed tracking
    ax = axes[0, 0]
    tc2 = all_data[2]
    ax.plot(tc2["t"], tc2["w_ref"], "k--", lw=1.1, label=r"$\omega^*$")
    ax.plot(tc2["t"], tc2["omega"], "#1f77b4", lw=1.1, label=r"$\omega$")
    ax.set_title("TC2 High-speed tracking", fontsize=9.5)
    ax.set_ylabel("Speed (rad/s)")
    ax.set_xlim(0, TSIM)
    ax.set_ylim(-5, 220)
    ax.legend(loc="lower right", frameon=True)
    ax.text(0.03, 0.90, "(a)", transform=ax.transAxes, fontweight="bold", fontsize=9)
    ax.text(
        0.50,
        0.24,
        "200 rad/s held with\npositive steady torque",
        transform=ax.transAxes,
        fontsize=7.5,
    )

    # Panel (b): TC3 Maximum-load boundary
    ax = axes[0, 1]
    tc3 = all_data[3]
    ax.plot(tc3["t"], tc3["w_ref"], "k--", lw=1.1, label=r"$\omega^*$")
    ax.plot(tc3["t"], tc3["omega"], "#d62728", lw=1.1, label=r"$\omega$")
    ax.axvline(LOAD_STEP_TIME, color="gray", ls=":", lw=0.9)
    ax.set_title("TC3 Maximum-load boundary", fontsize=9.5)
    ax.set_ylabel("Speed (rad/s)")
    ax.set_xlim(0, TSIM)
    ax.set_ylim(-5, 112)
    ax.legend(loc="lower left", frameon=True)
    ax.text(0.03, 0.90, "(b)", transform=ax.transAxes, fontweight="bold", fontsize=9)
    ax.text(
        0.46,
        0.24,
        "5 N·m step reaches the\ncontroller torque ceiling",
        transform=ax.transAxes,
        fontsize=7.5,
    )

    # Panel (c): TC4 Ramp tracking
    ax = axes[1, 0]
    tc4 = all_data[4]
    ax.plot(tc4["t"], tc4["w_ref"], "k--", lw=1.1, label=r"$\omega^*$")
    ax.plot(tc4["t"], tc4["omega"], "#2ca02c", lw=1.1, label=r"$\omega$")
    ax.set_title("TC4 Ramp tracking", fontsize=9.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (rad/s)")
    ax.set_xlim(0, TSIM)
    ax.set_ylim(-5, 112)
    ax.legend(loc="lower right", frameon=True)
    ax.text(0.03, 0.90, "(c)", transform=ax.transAxes, fontweight="bold", fontsize=9)
    ax.text(
        0.44,
        0.24,
        "Tuned PI keeps the ramp\nerror visibly small",
        transform=ax.transAxes,
        fontsize=7.5,
    )

    # Panel (d): TC5 Load-ramp adaptation
    ax = axes[1, 1]
    tc5 = all_data[5]
    ax.plot(tc5["t"], tc5["TL"], "k--", lw=1.1, label=r"$T_L$")
    ax.plot(tc5["t"], tc5["Te"], "#9467bd", lw=0.9, label=r"$T_e$")
    ax.set_title("TC5 Load-ramp adaptation", fontsize=9.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (N·m)")
    ax.set_xlim(0, TSIM)
    ax.set_ylim(-0.2, 6.2)
    ax.legend(loc="upper left", bbox_to_anchor=(0.13, 0.98), frameon=True)
    ax.text(0.03, 0.90, "(d)", transform=ax.transAxes, fontweight="bold", fontsize=9)
    ax.text(
        0.34,
        0.16,
        "Torque follows the rising load\nwhile speed stays near 100 rad/s",
        transform=ax.transAxes,
        fontsize=7.5,
    )

    _save_figure(fig, out_dir, "fig_operating_envelope")


def plot_validation_summary(
    tc1_data: dict[str, np.ndarray],
    tc1_metrics: dict[str, float],
    out_dir: Path,
) -> None:
    """Generate the 2-panel TC1 Simulink-vs-Python reproducibility figure."""
    _apply_plot_style()
    fig, (ax_left, ax_right) = plt.subplots(
        1,
        2,
        figsize=(8.8, 3.6),
        gridspec_kw={"width_ratios": [1.0, 1.38], "wspace": 0.18},
    )

    t = tc1_data["t"]
    mask = t <= 0.12
    sim_rise_s = SIMULINK_TC1_REF["rise_ms"] / 1e3
    py_rise_s = tc1_metrics["rise_ms"] / 1e3

    ax_left.plot(t[mask], tc1_data["omega"][mask], "#1f77b4", lw=1.2, label="Python rerun")
    ax_left.plot(t[mask], tc1_data["w_ref"][mask], "k--", lw=1.1, label=r"$\omega^*$")
    ax_left.axvline(sim_rise_s, color="#ff7f0e", ls=":", lw=1.1)
    ax_left.axvline(py_rise_s, color="#2ca02c", ls=":", lw=1.1)
    ax_left.text(
        0.045,
        20.0,
        f"archived Simulink\nrise = {SIMULINK_TC1_REF['rise_ms']:.1f} ms",
        color="#d95f02",
        fontsize=7.5,
    )
    ax_left.text(
        0.072,
        48.0,
        f"Python rerun\nrise = {tc1_metrics['rise_ms']:.1f} ms",
        color="#238b45",
        fontsize=7.5,
    )
    ax_left.text(0.03, 0.91, "(a)", transform=ax_left.transAxes, fontweight="bold", fontsize=9)
    ax_left.set_title("TC1 reproducibility check", fontsize=9.5)
    ax_left.set_xlabel("Time (s)")
    ax_left.set_ylabel("Speed (rad/s)")
    ax_left.set_xlim(0.0, 0.12)
    ax_left.set_ylim(0.0, 112.0)
    ax_left.legend(loc="lower right", frameon=True)

    # Panel (b): Comparison table
    ax_right.axis("off")
    ax_right.text(
        0.0,
        0.96,
        "(b) Archived Simulink vs. Python rerun",
        transform=ax_right.transAxes,
        fontweight="bold",
        fontsize=9.5,
    )

    spd_pct = abs(tc1_metrics["ss_speed"] - SIMULINK_TC1_REF["ss_speed"]) / (
        SIMULINK_TC1_REF["ss_speed"] / 100.0
    )
    rise_diff = abs(tc1_metrics["rise_ms"] - SIMULINK_TC1_REF["rise_ms"])
    trq_pct = abs(tc1_metrics["ss_torque"] - SIMULINK_TC1_REF["ss_torque"]) / (
        SIMULINK_TC1_REF["ss_torque"] / 100.0
    )
    cur_pct = abs(tc1_metrics["peak_i"] - SIMULINK_TC1_REF["peak_i"]) / (
        SIMULINK_TC1_REF["peak_i"] / 100.0
    )

    rows = [
        [
            "Speed (rad/s)",
            f"{SIMULINK_TC1_REF['ss_speed']:.2f}",
            f"{tc1_metrics['ss_speed']:.3f}",
            f"{spd_pct:.2f}%",
        ],
        [
            "Rise time (ms)",
            f"{SIMULINK_TC1_REF['rise_ms']:.1f}",
            f"{tc1_metrics['rise_ms']:.1f}",
            f"{rise_diff:.1f} ms",
        ],
        [
            "Mean torque (N·m)",
            f"{SIMULINK_TC1_REF['ss_torque']:.3f}",
            f"{tc1_metrics['ss_torque']:.3f}",
            f"{trq_pct:.2f}%",
        ],
        [
            "Peak current (A)",
            f"{SIMULINK_TC1_REF['peak_i']:.2f}",
            f"{tc1_metrics['peak_i']:.3f}",
            f"{cur_pct:.2f}%",
        ],
    ]

    table = ax_right.table(
        cellText=rows,
        colLabels=["Metric", "Simulink ref", "Python rerun", r"$\Delta$"],
        colWidths=[0.36, 0.22, 0.23, 0.19],
        cellLoc="center",
        loc="center",
        bbox=[0.0, 0.20, 1.0, 0.68],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(7.5)
    for (row_idx, col_idx), cell in table.get_celld().items():
        cell.set_edgecolor("#475569")
        cell.set_linewidth(0.7)
        if row_idx == 0:
            cell.set_facecolor("#e8eef7")
            cell.set_text_props(weight="bold")
        elif col_idx == 0:
            cell.set_text_props(ha="left")

    ax_right.text(
        0.0,
        0.06,
        "Source traceability: Simulink values from docs/Validation_Report.pdf;\n"
        "Python values from the current validation/srm_validation.py rerun.",
        transform=ax_right.transAxes,
        fontsize=7.2,
    )

    _save_figure(fig, out_dir, "fig_validation_summary")


# ─── Entry point ─────────────────────────────────────────────────────────────
def _print_tc1_validation(metrics: dict[str, float]) -> None:
    """Print TC1 back-to-back cross-validation comparison against Simulink."""
    print("\n  Cross-validation vs Simulink (TC1):")
    ref = SIMULINK_TC1_REF
    checks = [
        ("SS speed", abs(metrics["ss_speed"] - ref["ss_speed"]) / ref["ss_speed"] * 100, 1.0, "%"),
        ("Rise time", abs(metrics["rise_ms"] - ref["rise_ms"]), 15.0, "ms"),
        (
            "Mean torque",
            abs(metrics["ss_torque"] - ref["ss_torque"]) / ref["ss_torque"] * 100,
            2.0,
            "%",
        ),
        ("Peak i", abs(metrics["peak_i"] - ref["peak_i"]) / ref["peak_i"] * 100, 5.0, "%"),
    ]
    all_pass = True
    for name, delta, tol, unit in checks:
        passed = delta <= tol
        if not passed:
            all_pass = False
        status = "PASS" if passed else "FAIL"
        print(f"    {name:<16} delta = {delta:6.2f} {unit:<3}  tol <= {tol} {unit}  [{status}]")
    print(f"\n  Overall: {'ALL PASS' if all_pass else 'SOME FAIL'}")


def main(argv: Sequence[str] | None = None) -> None:
    """CLI entry point for running one or all SRM validation test cases."""
    parser = argparse.ArgumentParser(description="SRM validation script -- Group 13")
    parser.add_argument(
        "--tc",
        type=int,
        default=None,
        choices=[1, 2, 3, 4, 5],
        help="Test case to run (default: 1, or use --all)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run all 5 test cases and save individual and composite figures",
    )
    repo_root = Path(__file__).resolve().parent.parent
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=repo_root / "results",
        help="Directory for per-test-case figures (default: results/)",
    )
    parser.add_argument(
        "--paper-dir",
        type=Path,
        default=repo_root / "paper",
        help="Directory for composite paper figures (default: paper/)",
    )
    args = parser.parse_args(argv)

    if args.all:
        print("\nRunning all 5 test cases...")
        all_data: dict[int, dict[str, np.ndarray]] = {}
        all_metrics: dict[int, dict[str, float]] = {}
        for tc_num in range(1, 6):
            cfg = TEST_CASES[tc_num]
            print(f"\n{'=' * 60}")
            print(f"  SRM Validation -- {cfg['label']}")
            print(f"{'=' * 60}")
            data = run(tc_num)
            all_data[tc_num] = data
            all_metrics[tc_num] = compute_metrics(data, float(cfg["w_ref"]))
            plot(data, tc_num, args.results_dir)
        plot_operating_envelope(all_data, args.paper_dir)
        plot_validation_summary(all_data[1], all_metrics[1], args.paper_dir)
        print(f"\nAll figures saved to {args.results_dir} and {args.paper_dir}")
        return

    tc = args.tc if args.tc is not None else 1
    cfg = TEST_CASES[tc]

    print(f"\n{'=' * 60}")
    print(f"  SRM Validation -- {cfg['label']}")
    print(f"{'=' * 60}")

    data = run(tc)
    metrics = compute_metrics(data, float(cfg["w_ref"]))

    print(f"\n  {'Metric':<30} {'Value':>12}")
    print(f"  {'-' * 44}")
    print(f"  {'SS speed (rad/s)':<30} {metrics['ss_speed']:>12.4f}")
    print(f"  {'Rise time 10-90% (ms)':<30} {metrics['rise_ms']:>12.1f}")
    print(f"  {'Mean EM torque SS (N·m)':<30} {metrics['ss_torque']:>12.4f}")
    print(f"  {'Torque ripple RMS (N·m)':<30} {metrics['ripple_rms']:>12.4f}")
    print(f"  {'Peak phase current (A)':<30} {metrics['peak_i']:>12.3f}")

    if tc == 1:
        _print_tc1_validation(metrics)

    plot(data, tc, args.results_dir)


if __name__ == "__main__":
    main()
