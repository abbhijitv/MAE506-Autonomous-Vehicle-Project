# Modeling and Control of an Autonomous Vehicle Using Cruise Control and 2D Path Tracking

MAE 506: Advanced System Modeling, Dynamics and Control

## Project overview

This project combines mathematical modeling, system analysis, and feedback control of a vehicle in MATLAB. The final goal is to maintain vehicle speed while following straight and curved roads, evaluate stability and system properties, and compare controller performance.

The project includes nonlinear longitudinal dynamics and a 2D kinematic bicycle model. Planned extensions include cruise control, steering and path tracking, PID and state-feedback control, closed-loop simulations, and vehicle animation.

**Current work:** Stage 1 modeling and linearization are completed and locally tested. The current demo uses constant commanded force and zero steering to demonstrate straight-line motion. Closed-loop speed control and path tracking are not implemented. Stage 2 system analysis is next.

## Mathematical models

### Longitudinal model

```text
m*dv/dt = Fa - b*v - c*v*abs(v) - d
tau*dFa/dt = -Fa + u
```

The two states are speed `v` (m/s) and actual actuator force `Fa` (N). The input `u` is commanded drive/braking force, and `d` is opposing longitudinal disturbance force. Parameters `m`, `b`, `c`, and `tau` represent mass, linear resistance, quadratic drag, and actuator time constant.

The model is linearized about constant-speed cruising. Its state-space matrices describe deviations from the equilibrium, with `E` representing disturbance input. With the default cruising speed of 25 m/s and zero disturbance, the equilibrium actuator and commanded forces are both 1500 N:

```text
G(s) = 1 / (750*s^2 + 1535*s + 70)
```

### 2D kinematic bicycle model

```text
dX/dt = v*cos(psi)
dY/dt = v*sin(psi)
dpsi/dt = (v/L)*tan(delta)

Combined states: [X; Y; psi; v; Fa]
Commanded inputs: [u; delta]
```

`X` and `Y` are global rear-axle coordinates (m), `psi` is heading (rad), `L` is wheelbase (m), and `delta` is the front-wheel steering angle (rad). The five-state model retains the same longitudinal equations. The separate two-state subsystem remains available for SISO cruise-control analysis.

Reference paths include a straight road, `Y = 0`, and a sinusoidal road, `Y = A*sin(2*pi*X/lambda)`, with headings calculated from their slopes. Generating a path does not make the vehicle follow it.

All physical parameters, initial conditions, path settings, and demo inputs are configured in `config.m`. Defaults are illustrative modeling assumptions, not experimentally validated vehicle parameters.

## Stage progress

| Stage | Purpose | Status | Implementation details |
| --- | --- | --- | --- |
| 1 — Vehicle Modeling and Linearization | Establish and verify the physical models | Completed and locally tested | Nonlinear models, equilibrium, Jacobian, state-space and transfer-function models, reference paths, open-loop demo, plots, and validation checks |
| 2 — System Analysis | Evaluate system properties | Not implemented | `system_analysis.m` is a placeholder |
| 3 — Controller Design | Design speed and steering feedback | Not implemented | `pid_controller.m` and `state_feedback.m` are placeholders; observer and steering control are planned |
| 4 — Closed-Loop Simulations | Test integrated control scenarios | Not implemented | `run_simulations.m` is a placeholder; the existing open-loop demo belongs to Stage 1 |
| 5 — Performance Analysis and Validation | Evaluate and compare control methods | Not implemented | `compare_results.m` is a placeholder |
| 6 — Final Documentation and Presentation | Prepare final deliverables | Not implemented | Report, notes, references, and presentation files are placeholders |

Stage 1 local testing is reported by the project team. The source and generated PNG files are present; no new MATLAB execution was performed for this README update.

### Stage 1 — Vehicle Modeling and Linearization

- [x] Nonlinear longitudinal dynamics with aerodynamic drag and actuator dynamics
- [x] Equilibrium calculations and Jacobian linearization
- [x] State-space matrices `A`, `B`, `C`, `D`, `E` and transfer-function derivation
- [x] Five-state 2D kinematic bicycle model
- [x] Straight and sinusoidal reference paths with reference headings
- [x] Open-loop simulation using `ode45`
- [x] Vehicle trajectory, speed, and heading plots saved as PNG files
- [x] Local MATLAB validation, with silent checks in `main.m`

### Stage 2 — System Analysis

- [ ] Eigenvalues and stability analysis
- [ ] Controllability and observability rank tests and Gramians
- [ ] Controllable and observable canonical forms
- [ ] Coordinate transformations
- [ ] Open-loop response analysis

### Stage 3 — Controller Design

- [ ] PID cruise controller
- [ ] State-feedback controller using pole placement
- [ ] Integral action for steady-state tracking
- [ ] Luenberger state observer
- [ ] Steering and path-tracking controller

### Stage 4 — Closed-Loop Simulations

- [ ] Speed tracking and reference-speed changes
- [ ] Straight-road and curved-road tracking
- [ ] Uphill disturbance rejection
- [ ] Integrated speed and steering control
- [ ] Closed-loop MATLAB plots and vehicle animation

### Stage 5 — Performance Analysis and Validation

- [ ] Lyapunov stability analysis
- [ ] Rise time, settling time, and overshoot
- [ ] Steady-state tracking error and control effort
- [ ] Robustness testing with varying vehicle mass and drag
- [ ] PID versus state-feedback comparison

### Stage 6 — Final Documentation and Presentation

- [ ] Final MATLAB code review
- [ ] Technical report and references
- [ ] Final simulation figures and results
- [ ] Project presentation
- [ ] Team-member contributions
- [ ] Final README updates

## Project files

```text
MAE506-Autonomous-Vehicle/
├── README.md
├── .gitignore
├── config.m
├── main.m
├── vehicle_model.m
├── vehicle_2d_model.m
├── reference_path.m
├── vehicle_2d_demo.m
├── system_analysis.m
├── pid_controller.m
├── state_feedback.m
├── run_simulations.m
├── compare_results.m
├── tests/
│   └── smoke_test.m
├── figures/
│   ├── .gitkeep
│   ├── vehicle_trajectory.png
│   ├── vehicle_speed.png
│   └── vehicle_heading.png
├── docs/
│   ├── model_derivation.md
│   ├── analysis_notes.md
│   ├── controller_notes.md
│   └── results_notes.md
├── report/
│   ├── report_draft.md
│   └── references.txt
└── presentation/
    └── slide_outline.txt
```

| File or folder | Purpose and current status |
| --- | --- |
| `config.m` | Central vehicle, simulation, disturbance, actuator, and reference-path settings |
| `main.m` | Displays model results and runs silent equilibrium, dimension, transfer-function, and 2D checks |
| `vehicle_model.m` | Two-state nonlinear cruise model, equilibrium, linearization, and Control System Toolbox models |
| `vehicle_2d_model.m` | Five-state nonlinear bicycle dynamics with input validation |
| `reference_path.m` | Returns straight and curved path coordinates and headings |
| `vehicle_2d_demo.m` | Runs the open-loop simulation and exports three separate plots at 300 DPI |
| `figures/` | Generated simulation PNG files; `.gitkeep` preserves the folder |
| `system_analysis.m` | Empty placeholder for Stage 2 |
| `pid_controller.m`, `state_feedback.m` | Empty placeholders for controller design |
| `run_simulations.m`, `compare_results.m` | Empty placeholders for closed-loop scenarios and comparisons |
| `tests/smoke_test.m` | Empty test placeholder; current validation checks are in `main.m` |
| `docs/`, `report/`, `presentation/` | Existing files are empty placeholders for collaborative documentation |
| `.gitignore` | Empty placeholder for repository ignore rules |

## How to run

Requirements: MATLAB (developed with R2026b) and Control System Toolbox.

1. Open the repository folder in MATLAB and set it as the Current Folder.
2. Run the model summary and validation checks:

   ```matlab
   main
   ```

   This displays vehicle parameters, cruising equilibrium, state-space matrices, the physical-coefficient transfer function, and the 2D configuration. Checks are silent when successful and raise errors on failure.

3. Run the separate open-loop demonstration:

   ```matlab
   vehicle_2d_demo
   ```

   This simulates the vehicle with `ode45`, opens three plots, and saves them in `figures/`. Running `main` does not launch the demo. Rerunning the demo overwrites its three PNG files.

## Current simulation results

With the default configuration, the vehicle starts from rest at `(0, 0)` with zero heading and actuator force. A constant 1500 N command and zero disturbance cause speed to approach 25 m/s gradually; the demo does not impose instantaneous speed tracking. Zero steering keeps the heading at zero and produces a straight trajectory along positive X.

The generated plots are:

- [Vehicle trajectory](figures/vehicle_trajectory.png)
- [Vehicle speed](figures/vehicle_speed.png)
- [Vehicle heading](figures/vehicle_heading.png)

The curved reference path is available, but curved-road tracking, PID control, and state-feedback control are not implemented.

## Final project goals

- Autonomous speed regulation and 2D road following
- Stability, controllability, and observability analysis
- PID versus state-feedback control and performance comparison
- Disturbance rejection and robustness testing
- Integrated MATLAB simulations and vehicle animation

## Team collaboration

MATLAB source code is maintained in this GitHub repository. Teammates can review the current implementation and generated results here. Progress checklists will be updated as stages are implemented and tested. The report and presentation will be developed collaboratively, with team contributions documented in the final deliverables.
