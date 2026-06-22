# Changelog

All notable changes to this project are documented here.
Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning: [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] — 2026-06-22

### Added
- `src/SRM_params.m` — authoritative parameter file; builds 1000-point piecewise-linear
  inductance LUT and derivative LUT via `gradient()`
- `src/SRM_Project.slx` — five-layer Simulink model (PI Speed → Cubic TSF → T2I →
  Hysteresis → AHB → SRM Plant); ode4 fixed-step, Tₛ = 10 µs
- `src/SRM_plots.m` — post-processing script; exports speed, current, and torque figures
- `src/SRMcontrol.prj` / `src/buildfile.m` — MATLAB Project wrappers
- `validation/srm_validation.py` — independent Python (NumPy) re-implementation using
  forward-Euler at Tₛ = 10 µs; zero shared code with Simulink model; all 6 TC1
  cross-validation criteria pass (SS speed 0.01%, mean torque 0.000%, peak current 1.8%)
- `results/fig_tc1.pdf` / `.png` — TC1 baseline simulation figure (100 rad/s,
  1.5 N·m step at t = 0.1 s)
- `paper/main.tex` — full IEEE conference paper (IEEEtran `[conference]` class);
  12 sections covering theory, implementation, 5 TCs, validation, discussion, conclusion;
  TikZ inductance-profile figure, TikZ cascaded-control block diagram, TC1 results figure
- `paper/references.bib` — 9 BibTeX entries (Miller 1993 → MathWorks 2024)
- `docs/` — MS3 background report, MS4 submission, validation report, final report,
  final presentation
- `.gitignore` — excludes `slprj/`, `*.slxc`, `*.mat` cache, LaTeX build artefacts,
  Python bytecode, OS/editor files
- `README.md` — parameters table, inductance profile table, 5-TC table, validation
  results table, usage instructions, key equations, full reference list

### Changed
- Paper title updated from "Theoretical Foundations for Speed and Torque Control…"
  to "Speed and Torque Control of a Switched Reluctance Motor: MATLAB/Simulink
  Implementation, Cubic TSF Ripple Minimisation, and Independent Validation"
- Paper §VIII–§XII: replaced MS3 placeholder section ("no .m scripts or Simulink
  models available") with full implementation, results, and validation content

### Fixed
- `references.bib`: added missing `MathWorks2024` entry

---

## [0.3.0] — 2026-04-28 *(MS4 — Implementation)*

- Simulink model completed with all five test cases
- TC4 speed PI re-tuned: Kp = 15, Ki = 10 for ramp tracking
- Independent MATLAB validation script run; all metrics pass

## [0.2.0] — 2026-04-10 *(MS3 — Theoretical Background)*

- Full theoretical background report submitted
- LaTeX paper skeleton created on Overleaf (8 sections, no simulation results)
- `references.bib` populated with 8 entries

## [0.1.0] — 2026-03-10 *(MS1/MS2 — Project Setup)*

- Repository created
- Initial project topic: Speed and Torque Control of an SRM
- Group 13 team confirmed
