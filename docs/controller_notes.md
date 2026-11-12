# Stage 3 controller notes

## Integral state feedback

This is the primary course-related feedback controller. The standard longitudinal linearization uses `delta_x_dot = A*delta_x + B*delta_u` and `delta_y = C*delta_x + D*delta_u`, with `delta_x = x - x_eq` and `delta_u = u - u_eq`. Stability, controllability, and observability results concern this linearization about 25 m/s, not the complete nonlinear 2D model.

The optional disturbance term is `E*delta_d`, where `delta_d = d - d_eq` and `E = [-1/m; 0]`. With the default mass, `E = [-1/1500; 0]`. It describes how opposing force affects acceleration; it is not another control input or a separate major analysis requirement.

```text
delta_x = [v; Fa] - x_eq
z_dot = v_ref - v
A_aug = [A, zeros(2,1); -C, 0]
B_aug = [B; 0]
u_raw = u_eq - Kx*delta_x - Ki*z
```

`place` assigns augmented poles `[-0.25, -0.4, -1.5]` 1/s. The slower poles regulate speed, and the faster pole accounts for actuator lag without demanding a very fast force response. Default gains are approximately `Kx = [732.633333, 0.0516667]` and `Ki = -112.5`. The integral sign follows the equation above: positive speed error increases the command because `Ki` is negative.

The nonlinear demonstration starts from the configured speed and force, uses true states, and applies the same saturation and conditional anti-windup as PID. Pole placement is a local linear design, not a global stability guarantee for the saturated nonlinear plant.

## PID cruise control

PID is an additional practical cruise-control method for later comparison. With `e = v_ref - v`, the sampled command is:

```text
u_raw = F_ff + Kp*e + Ki*z - Kd*v_dot_filtered
F_ff = b*v_ref + c*v_ref*abs(v_ref) + d_eq
z_next = z + dt*e
```

Defaults are `Kp = 900 N/(m/s)`, `Ki = 120 N/m`, `Kd = 250 N/(m/s^2)`, and a 0.1 s derivative filter. The derivative is calculated from measured speed, avoiding reference derivative kick. Its filter weight is `exp(-dt/filter_tau)`. The command uses the current integral, then updates it for the next sample.

Both cruise controllers clamp commands to the existing ±6000 N limits. These illustrative limits correspond to 4 m/s² before resistance at rest. Conditional integration freezes the integral when saturation and speed error would drive force further beyond a limit, but allows unwinding in the opposite direction. The actuator force remains a plant state with its original lag. The two controllers have separate demonstrations; a formal comparison under identical scenarios is future work.

## Luenberger observer

The observer demonstrates estimation of internal speed and actuator-force states from measured speed and known commanded force. It is tested separately from the state-feedback loop.

```text
L = place(A', C', observer_poles)'
xhat_dot = A*(xhat - x_eq) + B*(u - u_eq) + L*(v - C*xhat)
```

Observer poles `[-2.5, -3.5]` 1/s are faster than the feedback poles. The default gain is approximately `L = [3.953333; 1125]`. Estimates are stored and plotted in actual coordinates, while the internal model uses deviations. The separate 20 s test uses the nonlinear plant initially at `[24; 1500]`, with estimates `[22; 1000]`, command 1500 N, and no disturbance increment.

Nonlinear drag introduces a small model mismatch away from 25 m/s. Finite-time errors need not be exactly zero. An unknown uphill disturbance is not included in the observer model; disturbance estimation, sensor noise, and observer-based feedback integration are outside this demonstration.

## Pure-pursuit steering

This additional controller provides automatic steering for curved-road tracking. It finds the nearest reference sample and selects a target 8 m farther along the sampled path's arc length. For target distance `ell` and target bearing relative to heading `alpha`:

```text
delta = atan2(2*L*sin(alpha), ell)
```

Steering is clamped to the configured ±30° limit. Lateral error is signed displacement along the nearest sample's normal. This is a sampled approximation to geometric cross-track error.

Both straight and sinusoidal roads are tested with a 1 m lateral offset and initial heading aligned with the reference heading. Forward speed is 10 m/s, with initial actuator force and constant command equal to its 540 N equilibrium. Thus the complete five-state plant runs unchanged, without imposing speed by overwriting a state. The simulation stops 8 m before the endpoint so its target stays ahead. The trajectory plot normally uses −10 to +10 m vertically, expanding if needed to include all data; this visual exaggeration does not change the simulation or tracking errors. The model assumes a rear-axle reference point and no tire slip; there is no steering actuator lag or moving-car animation. Integration with cruise control remains Stage 4 work.

Pure pursuit follows the geometric lookahead approach described in [MathWorks' pure-pursuit overview](https://www.mathworks.com/help/nav/ug/pure-pursuit-controller.html). The project implements the geometry directly and does not require Navigation Toolbox. State feedback and observer gains follow [MathWorks' pole-placement convention](https://www.mathworks.com/help/control/getstart/pole-placement.html).

## Numerical checks and current limits

Cruise and path commands update every 0.01 s and are held constant over a fourth-order Runge–Kutta step. The observer uses continuous speed feedback evaluated at the RK4 intermediate states. All design and scenario defaults are in `config.m`.

The project team reports successful local MATLAB R2026b execution of `main`, `vehicle_2d_demo`, and `run('tests/smoke_test.m')`. Reported final speed errors are approximately zero for the undisturbed PID and state-feedback runs, and 0.0001 m/s for PID with the 1500 N opposing step at 60 s. Observer final errors at 20 s are `9.656e-06 m/s` and `-5.697e-03 N`. Curved-road RMS lateral error over the entire run, including the initial correction, is 0.1467 m.

The smoke test requires final-five-second speed errors below 0.05 m/s, overshoot below 10%, force saturation compliance, and frozen integration during outward saturation. Observer tolerances over 15–20 s are 0.001 m/s and 1 N, allowing local model mismatch. After the first 30 m, path error must have RMS below 0.5 m and peak below 0.75 m. These are demonstration acceptance limits, not safety requirements or final performance rankings.

All twelve PNGs are MATLAB-generated exports under their stage folders. After plotting changes, rerun the MATLAB commands to regenerate the affected figures and check for regressions. Animation and integrated scenarios remain Stage 4 work; performance comparison remains Stage 5 work.
