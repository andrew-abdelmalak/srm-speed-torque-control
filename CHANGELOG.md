# Changelog

All notable changes to this project are documented here.
Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning: [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.1.0] — 2026-06-23

### Fixed
- `paper/main.tex`: Corrected Section X from “standalone MATLAB script” to
  “standalone Python script (validation/srm\_validation.py)”
- `paper/main.tex`: All result figures (Fig. 3–5) now correctly attributed to
  the independent Python re-implementation; captions updated to Python-measured
  values (peak current 7.09 A, rise time 36 ms)
- `paper/main.tex`: Moved `\usepackage{hyperref}` to after pgfplots (required
  load order)
- `paper/main.tex`: Removed unused `\usepackage{multirow}` and
  `\usetikzlibrary{fit}`
- `paper/references.bib`: All IEEE journal names abbreviated to IEEE standard
  (e.g., `IEEE Trans. Power Electron.`)
- `paper/fig_tc1.pdf`: Replaced with the version from `results/` (was a
  stale older render; now identical to the canonical result)
- `results/fig_tc1.pdf`: Regenerated — now matches `paper/fig_tc1.pdf`
- `.gitignore`: Removed stale `!src/*.mat` exception
- `README.md`: Updated `results/` and `paper/` file trees to reflect all five
  TC figures; corrected Overleaf usage instructions

### Added
- `paper/main.tex`: Paper organisation paragraph at end of Introduction
- `paper/main.tex`: Gap statement in Introduction before contribution list
- `paper/main.tex`: `\section*{Acknowledgment}` thanking Dr. Walid Omran
  and MCTR 908, GUC, Spring 2026
- `paper/main.tex`: `\includegraphics` for TC4 (Fig. 6) and TC5 (Fig. 7)
  with captions; paper now has **7 figures** total (2 TikZ + 5 photos)
- `paper/fig_tc4.pdf`, `paper/fig_tc5.pdf`: copies from `results/`
- `results/fig_tc4.pdf/.png`, `results/fig_tc5.pdf/.png`: TC4 and TC5
  simulation figures
- `CONTRIBUTING.md`: Contribution guide for the four-person team
- `matlab-code` branch deleted: was an unmerged development branch by
  Ahmed Mostafa containing an intermediate `SRM_Project.slx` (272 KB);
  the final version in `main/src/` (244 KB) is the authoritative submission.
  Deletion recorded here for provenance.

### Changed
- `paper/main.tex`: Subsection headings changed from `TC1 --- Baseline`
  style to `TC1: Baseline` (colon, more conventional in IEEE papers)
- `paper/main.tex`: TC4 and TC5 subsections now open with
  `Fig.~\ref{...} shows the TC\,N transient.` for consistency
- `paper/main.tex`: Table V caption: “Independent MATLAB” → “Independent
  Python”
- `paper/main.tex`: Index terms trimmed from 8 to 5 per IEEE standard
  (prior release already applied; confirmed here)
- README docs/ section: explicit milestone mapping added
- README `paper/` and `results/` trees updated to show all five TC figures

### Note on commit history
The ten binary-figure commits (`c85cff1b`–`152c0ca5`) added in the
previous release were pushed individually rather than as a single atomic
commit due to tool constraints. They are logically one change and should
be read as such.

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
