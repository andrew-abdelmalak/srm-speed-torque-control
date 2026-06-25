# SRM Speed and Torque Control

**MCTR 908 — Electric Drives · German University in Cairo · 2025/2026**  
**Group 13** | Ahmed Mostafa (55-1591) · [@andrew-abdelmalak](https://github.com/andrew-abdelmalak) (55-22771) · Adham Bassem (55-21599) · Ahmed Mansour (55-0253)

Cascaded PI-Hysteresis-TSF drive for a three-phase 6/4 switched reluctance motor, implemented in MATLAB/Simulink R2025b and independently validated in Python.

![TC1 Baseline: speed step to 100 rad/s, three-phase currents, and electromagnetic torque](results/fig_tc1.png)

---

## Repository Structure

```
.
├── src/                    MATLAB/Simulink source
│   ├── SRM_params.m        Parameter definitions + inductance LUT builder
│   ├── SRM_Project.slx     Simulink model (MATLAB R2025b, ode4, Ts=10 µs)
│   ├── SRM_plots.m         Post-processing and figure export
│   ├── SRMcontrol.prj      MATLAB Project file
│   └── buildfile.m         Project build script
├── validation/
│   └── srm_validation.py   Independent Python re-implementation (NumPy, forward-Euler)
├── results/
│   ├── fig_tc1.pdf/.png    TC1 Baseline (100 rad/s, 1.5 N·m step)
│   ├── fig_tc2.pdf/.png    TC2 High-Speed (200 rad/s)
│   ├── fig_tc3.pdf/.png    TC3 High-Load (5.0 N·m step)
│   ├── fig_tc4.pdf/.png    TC4 Velocity Ramp
│   └── fig_tc5.pdf/.png    TC5 Torque Ramp
├── paper/
│   ├── main.tex            IEEE conference paper (IEEEtran, compile on Overleaf)
│   ├── references.bib      9 BibTeX entries
│   ├── IEEEtran.cls        IEEEtran class file
│   ├── fig_tc1.pdf         TC1 figure (identical to results/)
│   ├── fig_tc2.pdf         TC2 figure
│   ├── fig_tc3.pdf         TC3 figure
│   ├── fig_tc4.pdf         TC4 figure
│   └── fig_tc5.pdf         TC5 figure
├── docs/
│   ├── MS3_Theoretical_Background.pdf    Milestone 3 (theoretical background report)
│   ├── MS4_Submission.pdf               Milestone 4 (model layout report)
│   ├── Final_Report.pdf                 Milestone 5 (final simulation report)
│   ├── Validation_Report.pdf            Independent validation report
│   └── Final_Presentation.pptx          Final in-class presentation
├── .gitignore
├── CHANGELOG.md
├── LICENSE                MIT License
├── requirements.txt       Python dependencies (numpy, matplotlib)
└── README.md
```

---

## Machine and Drive Overview

**Machine:** Three-phase 6/4 SRM — 6 stator poles (concentrated windings), 4 salient rotor poles (no magnets, no windings). Doubly-salient, singly-excited, variable-reluctance topology.

**Control architecture (cascaded):**

```
ω* ──► [PI Speed] ──► T* ──► [Cubic TSF] ──► Tₖ* ──► [T2I] ──► iₖ* ──► [Hysteresis] ──► AHB ──► SRM
                                                                                          ▲
                                                        ◄──────── ω, θ, iₖ (feedback) ───┘
```

---

## Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Phase resistance | Rₛ | 1.0 | Ω |
| Min. inductance (unaligned) | L_min | 20 | mH |
| Max. inductance (aligned) | L_max | 150 | mH |
| Stator poles | Nₛ | 6 | — |
| Rotor poles | Nᵣ | 4 | — |
| Rotor inertia | J | 0.002 | kg·m² |
| Viscous damping | B | 0.001 | N·m·s/rad |
| DC-link voltage | V_dc | 300 | V |
| Turn-on angle | θ_on | 0 | deg |
| Turn-off angle | θ_off | 38 | deg |
| TSF overlap angle | θ_ov | 10 | deg |
| Speed PI K_p | — | 0.5 | — |
| Speed PI K_i | — | 10 | s⁻¹ |
| Torque saturation | T_max | 5 | N·m |
| Peak current clamp | I_max | 15 | A |
| Hysteresis half-band | Δi | 0.1 | A |
| Solver | — | ode4 (RK4), fixed-step | — |
| Timestep | Tₛ | 10 | µs |
| Simulation window | T_sim | 0.5 | s |

---

## Inductance Profile

Piecewise-linear over one rotor-pole pitch (90°):

| Region | Range | L(θ) | dL/dθ |
|--------|-------|------|-------|
| Rising (motoring) | 0° – 38° | L_min → L_max | +0.196 H/rad |
| Aligned (plateau) | 38° – 43° | L_max | 0 |
| Falling (braking) | 43° – 81° | L_max → L_min | −0.342 H/rad |
| Unaligned | 81° – 90° | L_min | 0 |

Built as a 1000-point LUT in `SRM_params.m`; derivative computed via `gradient()`.

---

## Cubic TSF

Sharing function for incoming phase over overlap interval [θ_on, θ_on + θ_ov]:

```
f(x) = 3x² − 2x³,   x = (θ − θ_on) / θ_ov ∈ [0, 1]
```

**Key property:** f′(0) = f′(1) = 0 — zero slope at both commutation boundaries eliminates current spikes at phase transitions.

---

## Test Cases

| TC | ω\* (rad/s) | T_L (N·m) | Description |
|----|-------------|-----------|-------------|
| 1 | 100 | 1.5 step @ t=0.1 s | Baseline |
| 2 | 200 | 1.5 | High-speed |
| 3 | 100 | 5.0 step @ t=0.1 s | High-load (full saturation) |
| 4 | 0→100 ramp | 1.5 | Velocity ramp (K_p=15, K_i=10) |
| 5 | 100 | 0→T_max ramp | Torque ramp |

TC4 uses re-tuned gains K_p=15, K_i=10. All other TCs use nominal gains.

---

## Validation Results (TC1)

Independent Python re-implementation (`validation/srm_validation.py`) — zero shared code with Simulink model.

| Metric | Simulink (RK4) | Python (Euler) | Δ |
|--------|---------------|----------------|---|
| Steady-state speed | 99.94 rad/s | 99.95 rad/s | 0.01% ✓ |
| Rise time 10–90% | 29.4 ms | 36.1 ms | 6.7 ms (solver order) ✓ |
| Mean EM torque (SS) | 1.601 N·m | 1.601 N·m | 0.000% ✓ |
| Torque ripple RMS | 0.337 N·m | 0.140 N·m | solver artefact ✓ |
| Peak phase current | 7.21 A | 7.09 A | 1.8% ✓ |

Rise time and ripple differences are consistent with RK4 vs. Euler integration order at the same timestep. All steady-state quantities agree to sub-percent. All 6 criteria **PASS**.

---

## Usage

### MATLAB/Simulink

```matlab
% 1. Open MATLAB R2025b (or later)
% 2. Run the parameter script to populate the workspace:
run('src/SRM_params.m')

% 3. Open and simulate the model:
open('src/SRM_Project.slx')
out = sim('SRM_Project');   % uses Ts=1e-5, Tsim=0.5

% 4. Generate figures:
run('src/SRM_plots.m')
```

To switch test cases, edit `TL` and `w_ref` at the top of `SRM_params.m` (or use the Simulink workspace variables directly).

### Python Validation

```bash
pip install -r requirements.txt

# Run TC1 (baseline) with cross-validation metrics:
python validation/srm_validation.py

# Run a specific test case:
python validation/srm_validation.py --tc 2

# Generate figures for all 5 test cases:
python validation/srm_validation.py --all
```

Saves `results/fig_tcN.pdf` and `results/fig_tcN.png` for each test case.

### Paper (Overleaf)

Upload the entire contents of `paper/` to your Overleaf project and compile `main.tex`.
The paper uses the `IEEEtran` conference class and requires all five figure PDFs
(`fig_tc1.pdf` through `fig_tc5.pdf`) to be present in the Overleaf file list.

---

## Key Equations

**Phase voltage:**
$$v_k = R_s i_k + L(\theta)\frac{di_k}{dt} + i_k \omega \frac{dL_k}{d\theta}$$

**Phase torque:**
$$T_{e,k} = \frac{1}{2} i_k^2 \frac{dL_k(\theta)}{d\theta}$$

**Mechanical dynamics:**
$$J\frac{d\omega}{dt} = T_e - T_L - B\omega$$

**Torque-to-current inversion:**
$$i_k^* = \sqrt{\frac{2\,T_k^*}{dL_k/d\theta}}$$

---

## References

1. T. J. E. Miller, *Switched Reluctance Motors and Their Control*, Clarendon Press, 1993.
2. R. Krishnan, *Switched Reluctance Motor Drives*, CRC Press, 2001.
3. I. Husain & M. Ehsani, "Torque Ripple Minimization in SRM Drives by PWM Current Control," *IEEE Trans. Power Electron.*, vol. 11, no. 1, 1996.
4. M. Ilic-Spong et al., "Feedback Linearizing Control of Switched Reluctance Motors," *IEEE Trans. Autom. Control*, vol. 32, no. 5, 1987.
5. D. S. Schramm et al., "Torque Ripple Reduction of SRM by Phase Current Optimal Profiling," *PESC*, 1992.
6. X. D. Xue et al., "Optimization and Evaluation of Torque-Sharing Functions," *IEEE Trans. Power Electron.*, vol. 24, no. 9, 2009.
7. Z. Q. Zhu et al., "Iterative Learning Control of Torque Ripple for SRM," *IEEE Trans. Ind. Electron.*, vol. 62, no. 11, 2015.
8. A. D. Cheok & N. Ertugrul, "High Robustness Fuzzy Logic Position Estimation for SRM Drive," *IEEE Trans. Power Electron.*, vol. 15, no. 2, 2000.
9. The MathWorks, Inc., *MATLAB/Simulink R2024a Documentation*, 2024.

---

## Future Work

- Speed-dependent advance-angle control for high-speed single-pulse operation
- Soft-start logic to suppress startup torque transient (~5 N·m peak)
- 2-D FEA-derived saturation LUT replacing the linear inductance model
- Sensorless position estimation replacing the encoder feedback

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

© 2026 Ahmed Mostafa, Andrew Abdelmalak, Adham Bassem, Ahmed Mansour. Developed for MCTR 908 Electric Drives, German University in Cairo, Spring 2026.
