"""Unit, physics-invariant, and regression tests for the 6/4 SRM drive model."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest
import srm_validation as srm

REPO_ROOT = Path(__file__).resolve().parent.parent
SRM_PARAMS_M = REPO_ROOT / "src" / "SRM_params.m"


def _parse_matlab_scalars(path: Path) -> dict[str, float]:
    """Extract numeric and deg2rad scalar assignments from src/SRM_params.m."""
    text = path.read_text(encoding="utf-8")
    values: dict[str, float] = {}
    for raw_line in text.splitlines():
        line = raw_line.split("%", 1)[0].strip()
        if not line or "=" not in line:
            continue
        match = re.match(r"^([A-Za-z_]\w*)\s*=\s*([^;]+);?$", line)
        if not match:
            continue
        name, expr = match.group(1), match.group(2).strip()
        deg_match = re.fullmatch(r"deg2rad\(\s*([+-]?\d+(?:\.\d+)?)\s*\)", expr)
        if deg_match:
            values[name] = float(np.deg2rad(float(deg_match.group(1))))
            continue
        try:
            values[name] = float(expr)
        except ValueError:
            continue
    return values


def test_matlab_python_parameter_consistency() -> None:
    """Verify Python constants match src/SRM_params.m."""
    m = _parse_matlab_scalars(SRM_PARAMS_M)
    assert m["Rs"] == pytest.approx(srm.RS)
    assert m["Lmin"] == pytest.approx(srm.LMIN)
    assert m["Lmax"] == pytest.approx(srm.LMAX)
    assert m["Nph"] == pytest.approx(srm.NPH)
    assert m["Nr"] == pytest.approx(srm.NR)
    assert m["theta_on"] == pytest.approx(0.0)
    assert m["theta_off"] == pytest.approx(srm.TH_OFF)
    assert m["theta_ov"] == pytest.approx(srm.THETA_OV)
    assert m["J"] == pytest.approx(srm.J)
    assert m["B"] == pytest.approx(srm.B_FRIC)
    assert m["Vdc"] == pytest.approx(srm.VDC)
    assert m["Imax"] == pytest.approx(srm.IMAX)
    assert m["Kp_w"] == pytest.approx(srm.KP_W)
    assert m["Ki_w"] == pytest.approx(srm.KI_W)
    assert m["T_max"] == pytest.approx(srm.T_MAX)
    assert m["Ts"] == pytest.approx(srm.TS)
    assert m["Tsim"] == pytest.approx(srm.TSIM)
    assert m["delta_i"] == pytest.approx(srm.DELTA_I)


def test_inductance_lut_invariants() -> None:
    """Verify periodicity, bounds, and piecewise slope signs of L(theta)."""
    angles = np.linspace(0.0, srm.TAU, 360, endpoint=False)
    inductances = np.array([srm.lut(float(th))[0] for th in angles])

    assert np.all(inductances >= srm.LMIN - 1e-12)
    assert np.all(inductances <= srm.LMAX + 1e-12)

    for th in [0.1, 0.4, 0.8, 1.2]:
        l0, dl0 = srm.lut(th)
        l1, dl1 = srm.lut(th + srm.TAU)
        assert l0 == pytest.approx(l1)
        assert dl0 == pytest.approx(dl1)

    # Interior of rising [0, 38], flat [38, 43], falling [43, 81], unaligned [81, 90] deg
    _, dl_rise = srm.lut(float(np.deg2rad(19.0)))
    _, dl_flat = srm.lut(float(np.deg2rad(40.5)))
    _, dl_fall = srm.lut(float(np.deg2rad(62.0)))
    _, dl_unaligned = srm.lut(float(np.deg2rad(85.5)))

    assert dl_rise > 0.15
    assert abs(dl_flat) < 1e-6
    assert dl_fall < -0.15
    assert abs(dl_unaligned) < 1e-6


def test_cubic_tsf_invariants() -> None:
    """Verify cubic boundary slopes, monotonicity, and three-phase partition of unity."""
    assert srm.cubic(0.0) == pytest.approx(0.0)
    assert srm.cubic(0.5) == pytest.approx(0.5)
    assert srm.cubic(1.0) == pytest.approx(1.0)

    eps = 1e-6
    assert (srm.cubic(eps) - srm.cubic(0.0)) / eps == pytest.approx(0.0, abs=1e-5)
    assert (srm.cubic(1.0) - srm.cubic(1.0 - eps)) / eps == pytest.approx(0.0, abs=1e-5)

    xs = np.linspace(0.0, 1.0, 201)
    vals = np.array([srm.cubic(float(x)) for x in xs])
    assert np.all(np.diff(vals) >= -1e-15)

    # Partition of unity across the entire rotor pole pitch [0, 90 deg)
    thetas = np.linspace(0.0, srm.TAU, 900, endpoint=False)
    for th in thetas:
        total = sum(srm.tsf_fraction(float((th - phi) % srm.TAU)) for phi in srm.PHI_K)
        assert total == pytest.approx(1.0, abs=1e-12)


def test_step_count_and_reference_trajectory() -> None:
    """Verify exact 50000 simulation steps (5000 decimated samples) and w_ref array."""
    tc1 = srm.run(1)
    assert len(tc1["t"]) == 5000
    assert tc1["t"][0] == pytest.approx(0.0)
    assert tc1["t"][-1] == pytest.approx(0.4999)
    assert "w_ref" in tc1
    assert len(tc1["w_ref"]) == 5000
    assert np.allclose(tc1["w_ref"], 100.0)

    tc4 = srm.run(4)
    assert len(tc4["t"]) == 5000
    assert "w_ref" in tc4
    assert tc4["w_ref"][0] == pytest.approx(0.0)
    assert tc4["w_ref"][500] == pytest.approx(50.0, abs=0.2)
    assert tc4["w_ref"][-1] == pytest.approx(100.0)


def test_tc1_cross_validation_regression() -> None:
    """Verify TC1 matches archived Simulink reference metrics within tolerance."""
    data = srm.run(1)
    metrics = srm.compute_metrics(data, srm.TEST_CASES[1]["w_ref"])

    assert metrics["ss_speed"] == pytest.approx(100.0, rel=1e-3)
    assert abs(metrics["ss_speed"] - 99.94) / 99.94 <= 0.01
    assert abs(metrics["rise_ms"] - 29.4) <= 15.0
    assert metrics["ss_torque"] == pytest.approx(1.600, rel=2e-3)
    assert metrics["peak_i"] <= srm.IMAX
    assert abs(metrics["peak_i"] - 7.21) / 7.21 <= 0.05
    assert 0.0 < metrics["ripple_rms"] < 0.5


def test_operating_envelope_tc2_to_tc5() -> None:
    """Verify physics and control boundaries across TC2-TC5."""
    # TC2: 200 rad/s high speed stays in motoring mode
    tc2 = srm.run(2)
    m2 = srm.compute_metrics(tc2, srm.TEST_CASES[2]["w_ref"])
    assert m2["ss_speed"] == pytest.approx(200.0, abs=0.05)
    ss_mask = tc2["t"] > 0.8 * tc2["t"][-1]
    assert tc2["Te"][ss_mask].min() > 1.0

    # TC3: 5.0 N.m load hits torque clamp T_MAX stably near 79.6 rad/s
    tc3 = srm.run(3)
    m3 = srm.compute_metrics(tc3, srm.TEST_CASES[3]["w_ref"])
    assert m3["ss_torque"] == pytest.approx(5.0, abs=0.05)
    assert m3["ss_speed"] == pytest.approx(79.6, abs=0.5)

    # TC4: 0->100 rad/s ramp in 0.1 s with Kp=15
    tc4 = srm.run(4)
    ramp_mask = tc4["t"] <= 0.1
    ramp_err = tc4["w_ref"][ramp_mask] - tc4["omega"][ramp_mask]
    ramp_rmse = float(np.sqrt(np.mean(ramp_err**2)))
    ramp_max = float(np.max(np.abs(ramp_err)))
    assert ramp_rmse < 0.25
    assert ramp_max < 1.30

    # TC5: continuous 0->5 N.m load ramp drives monotonic current growth
    tc5 = srm.run(5)
    m5 = srm.compute_metrics(tc5, srm.TEST_CASES[5]["w_ref"])
    assert m5["ss_speed"] > 98.5
    t = tc5["t"]
    i_max = tc5["currents"].max(axis=1)
    peak_early = float(i_max[(t >= 0.10) & (t < 0.15)].max())
    peak_mid = float(i_max[(t >= 0.25) & (t < 0.30)].max())
    peak_late = float(i_max[(t >= 0.45) & (t <= 0.50)].max())
    assert peak_early < peak_mid < peak_late


def test_composite_figure_generators_exist_and_write(tmp_path: Path) -> None:
    """Verify all 7 figure pairs (TC1-5 + 2 composite paper figures) can be generated."""
    assert hasattr(srm, "plot_operating_envelope")
    assert hasattr(srm, "plot_validation_summary")

    results_dir = tmp_path / "results"
    paper_dir = tmp_path / "paper"
    srm.main(["--all", "--results-dir", str(results_dir), "--paper-dir", str(paper_dir)])

    for tc in range(1, 6):
        assert (results_dir / f"fig_tc{tc}.pdf").stat().st_size > 1000
        assert (results_dir / f"fig_tc{tc}.png").stat().st_size > 1000

    for stem in ("fig_operating_envelope", "fig_validation_summary"):
        assert (paper_dir / f"{stem}.pdf").stat().st_size > 1000
        assert (paper_dir / f"{stem}.png").stat().st_size > 1000
