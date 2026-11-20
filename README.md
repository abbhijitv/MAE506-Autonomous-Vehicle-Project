# Autonomous Vehicle Cruise Control — PID vs. State-Feedback Control

**MAE 506 — Advanced System Modeling, Dynamics and Control**

## Overview

This project models longitudinal vehicle motion and compares exactly two speed controllers: the existing PID implementation in `code/pid_controller.m` and Ankit Dhara's pole-placement state-feedback design, preserved in the standalone script `code/cruisecontrol_minimalcode4.m`. Both operate on the same nonlinear vehicle model under identical reference, initial-state, time-grid, actuator, and force-limit conditions.

`code/main.m` is the single entry point. It displays the model, performs system analysis, runs the controller comparison, exports figures, and prints performance metrics. Earlier project code and outputs are preserved in the local, Git-ignored `archive/` folder and are not needed to run a fresh clone.

## Getting Started

Requirements: **MATLAB R2026b** and **Control System Toolbox**. Set MATLAB's Current Folder to the repository's `code/` folder and run:

```matlab
main
```

Run the project checks in a fresh MATLAB session or batch mode:

```matlab
run('tests/smoke_test.m')
```

Running `main` saves exactly six files in `results/`: five numbered PNGs at 300 DPI and `simulation_results.txt`. The folder is created automatically, and all six files are overwritten with the current run results. In interactive MATLAB sessions, the five comparison figures remain visible. In `-batch` runs, they stay hidden and close after export. Rerunning the comparison closes only its previously tagged figure windows, leaving unrelated figures alone.

All active MATLAB code and tests are in `code/`. The six output files remain in the repository-root `results/` folder, regardless of the current working folder. Numerical results remain available in `model`, `analysis`, `controller`, and `results`. No archive folders need to be added to the MATLAB path.

### Original Standalone Script

To run Ankit's original work independently:

```matlab
cruisecontrol_minimalcode4
```

This is the exact original script, including its original system analysis, phase portrait, response plots, comments, reference gain, and formatting. It contains `clear` and `close all`, so run it separately from the final comparison; it resets the workspace and closes existing figures. `main` never executes this script. The standalone script's original plots and scaling are separate from the five final comparison outputs.

From the repository root, the equivalent batch commands are:

```bash
/Applications/MATLAB_R2026b.app/bin/matlab -batch "run('code/main.m')"
/Applications/MATLAB_R2026b.app/bin/matlab -batch "run('code/tests/smoke_test.m')"
```

## Longitudinal Vehicle Model

The original nonlinear equations are unchanged:

```text
m*dv/dt = Fa - b*v - c*v*abs(v) - d
tau*dFa/dt = -Fa + u
```

The state is `x = [v; Fa]`, where `v` is absolute vehicle speed in m/s and `Fa` is actual actuator force in N. The input `u` is commanded drive/braking force in N. Actuator lag means `Fa` generally differs from `u` during transients. The optional opposing force `d` remains part of the model; the final comparison uses `d = d_eq = 0` without disturbance steps.

| Setting | Default |
| --- | --- |
| Vehicle mass, `m` | 1500 kg |
| Linear resistance, `b` | 50 N·s/m |
| Quadratic drag coefficient, `c` | 0.4 N·s²/m² |
| Actuator time constant, `tau` | 0.5 s |
| Reference and linearization speed | 25 m/s |
| Initial speed and actuator force | 0 m/s and 0 N |
| Simulation duration and time step | 120 s and 0.01 s |
| Commanded-force limits | −6000 to +6000 N |

All settings are defined centrally in `code/config.m`. They are illustrative modeling assumptions, not experimentally validated vehicle parameters.

### Equilibrium and Linearization

```text
Fa_eq = b*v_eq + c*v_eq*abs(v_eq) + d_eq
u_eq = Fa_eq
x_eq = [v_eq; Fa_eq]

delta_x = x - x_eq
delta_u = u - u_eq
delta_y = y - v_eq

delta_x_dot = A*delta_x + B*delta_u
delta_y = C*delta_x + D*delta_u
```

At the default equilibrium, `x_eq = [25; 1500]` and `u_eq = 1500 N`. The force-to-speed SISO linearization is:

```text
A = [-(b + 2*c*abs(v_eq))/m, 1/m;
      0,                    -1/tau]
B = [0; 1/tau]
C = [1, 0]
D = 0

G(s) = 1 / (750*s^2 + 1535*s + 70)
```

The optional disturbance matrix is `E = [-1/m; 0]`, acting on `delta_d = d - d_eq`. Standard stability, controllability, and observability analysis uses `A`, `B`, `C`, and `D`. Plotting adds equilibrium values back so speed axes show absolute m/s, not deviations.

## System Analysis

`code/system_analysis.m` retains the existing eigenvalue, controllability, observability, Gramian, and canonical-form calculations. Similarity transformations and transfer-function equivalence are checked numerically. It does not create figures. The original phase portrait remains available only by running the preserved standalone script separately.

| Property | Default result |
| --- | --- |
| Open-loop eigenvalues | −0.0466667 and −2 s⁻¹ |
| Open-loop equilibrium | Asymptotically stable |
| Controllability rank | 2/2 |
| Observability rank | 2/2 |
| Controllability and observability Gramians | Positive definite |
| Minimum controllability Gramian eigenvalue | Approximately `4.547225e-06` |
| Minimum observability Gramian eigenvalue | Approximately `2.652548e-08` |
| Canonical forms | Numerically verified |

These properties apply to the longitudinal linearization about the cruise equilibrium. They do not establish global stability of the nonlinear plant with force saturation.

## Controllers

### Existing PID Controller

`code/pid_controller.m` is preserved without changing its code or gains. It uses equilibrium/feedforward force, a filtered measured-speed derivative, force saturation, and conditional-integration anti-windup.

```text
e = v_ref - v
F_ff = b*v_ref + c*v_ref*abs(v_ref) + d_eq
u_raw = F_ff + Kp*e + Ki*z - Kd*v_dot_filtered
```

Defaults are `Kp = 900`, `Ki = 120`, `Kd = 250`, and a 0.1 s derivative-filter time constant. The integral updates only when speed error does not drive the command farther into saturation.

### Ankit's Pole-Placement State Feedback

**`code/cruisecontrol_minimalcode4.m` is restored byte for byte to the original script**, matching Git history and the byte-preserved reference `code/tests/fixtures/cruisecontrol_minimalcode4.original`. It is not a function and has not been edited to integrate it with `main`.

For the comparison only, `code/control_simulations.m` reproduces the same specifications, pole calculations, and `place` call using the shared model. It returns the computed design in `controller`, without calling the original script or adding a separate controller file.

```text
Settling_Time = 2 s
Overshoot = 7.5 percent
OS = Overshoot/100
zeta = -log(OS)/sqrt(pi^2 + log(OS)^2)
wn = 4/(zeta*Settling_Time)
desiredPoles = -zeta*wn +/- 1i*wn*sqrt(1-zeta^2)
K = place(A, B, desiredPoles)
Acl = A - B*K
```

The resulting desired poles are approximately `-2 +/- 2.425690i` s⁻¹. With defaults, `K ≈ [7274.612521, 0.9766667]` and the corrected reference gain is approximately `7412.979188` N/(m/s).

The command uses actual states consistently with the deviation model:

```text
u_raw = u_eq - K*(x - x_eq) + Gain*(v_ref - v_eq)
u = min(max(u_raw, F_min), F_max)
```

There is **no integral action** in this controller. The previous integral state-feedback implementation is archived and is not called by the final project.

#### Reference-Gain Correction

The original script still uses, unchanged:

```matlab
Gain = -OpenLoop_Gain/((C*(Acl^-1)*B));
```

That makes the reference-to-speed DC gain equal to `OpenLoop_Gain`, about `1/70`, rather than unity. Only the integration block in `code/control_simulations.m` uses corrected reference scaling, with a linear solve instead of an explicit inverse:

```matlab
Gain = -1/(C*(Acl\B));
```

For the existing `D = 0` model, this gives unity steady-state reference tracking in the linearization. It does not guarantee exact nonlinear tracking for every reference far from the chosen equilibrium. The default target equals the equilibrium speed, so both controllers have the same physical 25 m/s objective.

The 2 s settling time and 7.5% overshoot are unsaturated linear design specifications, not promised results for acceleration from rest with force limits. Final comparison metrics are calculated from the actual nonlinear simulation histories.

## Simulation and Performance Metrics

`code/control_simulations.m` reuses the earlier sampled nonlinear cruise simulation and RK4 integration. Each command is held constant for one 0.01 s sample. Both controllers start at `[v0; Fa0] = [0; 0]` and run for 120 s with the same ±6000 N limits. The open-loop comparison uses the same initial states and plant, with a constant target-equilibrium force of 1500 N; it has no feedback.

| Metric | Common definition |
| --- | --- |
| Rise time | Time between the first sampled 10% and 90% crossings of the requested change from initial speed to target |
| Settling time | First sample after the last excursion outside ±2% of the requested speed change |
| Overshoot | Peak excess beyond the target, as a percentage of the requested speed change; minimum 0% |
| Steady-state error | Target minus mean absolute vehicle speed over the final 5 s; a signed value in m/s |
| Peak control input | Maximum absolute commanded force `u`, in N; not actual actuator force `Fa` |

Unreached rise thresholds or an unsettled response produce `NaN` time metrics rather than an invented result. Default-case smoke tests require finite metrics and final-window tracking within 0.05 m/s.

### Current Validation Status

The revised `main` and smoke test still require local MATLAB execution. The installed executable fails at startup in the coding environment with the known Qt/NEON processor error, before any project code runs. Thus the new MATLAB plots and runtime results have not been generated or verified here.

Independent NumPy simulations of the same default nonlinear equations and sampled commands give the following preliminary cross-checks. These are **not MATLAB test results** and do not replace the local smoke test:

| Controller | Rise time (s) | Settling time (s) | Overshoot (%) | Steady-state error (m/s) | Peak command (N) |
| --- | --- | --- | --- | --- | --- |
| PID | 5.90 | 15.77 | 2.848 | Approximately 0 | 6000 |
| Ankit state feedback | 5.75 | 7.50 | 0.617 | Approximately 0 | 6000 |

## Saved Results

On a working MATLAB installation, `main` saves exactly the following five PNGs and one plain-text report under `results/`. It does not export a phase portrait or a sixth plot. Previous output files are preserved under `archive/previous_project/` and are not substitutes for these comparison outputs.

| Output | Content |
| --- | --- |
| `results/01_pid_speed_tracking.png` | PID absolute vehicle speed and the desired speed reference |
| `results/02_state_feedback_speed_tracking.png` | State-feedback absolute vehicle speed and the desired reference |
| `results/03_pid_vs_state_feedback.png` | Both controlled speeds and the common reference |
| `results/04_open_loop_vs_closed_loop.png` | Open-loop and state-feedback speed on the same physical scale |
| `results/05_control_effort.png` | Both saturated actuator commands `u` in N, with the common force limits |
| `results/simulation_results.txt` | Current-run parameters, equilibrium, matrices, transfer coefficients, system analysis, controller design, individual metrics, and comparison table |

The plain-text report includes units and scientific notation for small values. It records open- and closed-loop classifications, full controllability/observability matrices and ranks, PID gains, state-feedback specifications and poles, the feedback/reference gains, and all five performance metrics for both controllers. Values come from the current MATLAB run, not from the preliminary table above. Its simple headings and aligned text can be selected and copied into Google Docs.

The report is opened in overwrite mode and closed using a function-scoped cleanup handler, including when an error interrupts writing. A run timestamp and file-update checks help detect stale outputs. Unexpected extra files cause validation to fail rather than being silently deleted.

## Repository Structure

| File or folder | Purpose |
| --- | --- |
| `code/main.m` | Single entry point for modeling, analysis, comparison, and metrics |
| `code/config.m` | Shared physical parameters, controller specifications, and metric settings |
| `code/vehicle_model.m` | Unchanged nonlinear longitudinal plant, equilibrium, and linear models |
| `code/system_analysis.m` | System-property checks and retained Gramians/canonical forms; no figure exports |
| `code/pid_controller.m` | Existing PID implementation, unchanged |
| `code/cruisecontrol_minimalcode4.m` | Exact original standalone script; run separately from `main` |
| `code/control_simulations.m` | Minimal design integration, shared nonlinear simulations, five result figures, and metrics |
| `code/tests/smoke_test.m` | Model, reference-scaling, comparison-condition, tracking, and export checks |
| `code/tests/fixtures/` | Original controller reference bytes used only by the preservation tests |
| `results/` | Exactly five numbered PNGs and `simulation_results.txt` |
| `archive/previous_project/` | Local-only historical source and notes, excluded from Git |
| `archive/previous_project/figures/` | Local-only prior plot outputs, excluded from Git |
| `report/`, `presentation/` | Existing draft placeholders for the final deliverables |

Do not add `archive/` recursively to the MATLAB path: its historical functions deliberately retain their original names.

## Testing

The updated smoke test runs `main` and checks the expected matrices, equilibrium, transfer function, eigenvalues, controllability, observability, Lyapunov residuals, corrected unity reference scaling, and absence of state-feedback integral action. It verifies common initial states/references/time grids, finite histories, commanded and actual force bounds, PID anti-windup, final tracking, and metric consistency. Sampled plant steps are cross-checked against `ode45`. The pole-placement gains are independently checked from the original performance specifications. The original standalone script and PID source are compared with the byte-preserved reference files in `code/tests/fixtures/`, so this test also works in a fresh clone without the local archive. Exactly the five named PNGs and numerical text report must exist in `results/`, with no extra files. Report matrices and metrics are checked against the current calculated values, file-update times are checked for the current run, and the report must be closed after writing. Batch figures must close; interactive comparison figures must remain visible.

From the `code/` folder, run locally before using the final comparison in the report:

```matlab
main
run('tests/smoke_test.m')
```

## Team and Contributions

The project has four members. Ankit Dhara's state-feedback contribution remains identifiable in `code/cruisecontrol_minimalcode4.m`; the existing PID contribution remains unchanged in `code/pid_controller.m`. The earlier complete responsibility plan is preserved in `archive/previous_project/README.md`. The following report and presentation assignments describe the final cruise-control scope rather than a verified history of every code change.

| Team Member | Technical Focus | Final Report | Presentation |
| --- | --- | --- | --- |
| Abbhijit Venkatachalam | Longitudinal modeling, project integration, and integration testing | Introduction, system modeling, equations and assumptions | Project overview and vehicle model |
| Logan Deison | Stability, controllability, observability, Gramians, canonical forms, and mathematical verification | System analysis and validation results | System properties and mathematical results |
| Tyler Winter | PID control, tuning analysis, and controller performance comparisons | Controller design and performance evaluation | PID method and comparison results |
| Ankit Dhara | Pole-placement state-feedback controller and simulation visualization | Simulation results, discussion, and conclusions | State-feedback method, simulation results, and conclusions |

All four members share MATLAB implementation, debugging, testing, technical documentation, report review, presentation preparation, and final-deliverable review. The intended overall workload is approximately 25% per member, with balanced report writing and about one-quarter of the presentation each. These assignments do not imply that unperformed work is complete.
