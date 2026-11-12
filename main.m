clc;
clear;

% load the configuration and create the vehicle model
cfg = config();
model = vehicle_model(cfg);

% display the vehicle parameters
fprintf('Stage 1 - Vehicle Model\n');
fprintf('Mass: %.2f kg\n', cfg.m);
fprintf('Linear resistance coefficient: %.2f N*s/m\n', cfg.b);
fprintf('Quadratic drag coefficient: %.2f N*s^2/m^2\n', cfg.c);
fprintf('Actuator time constant: %.2f s\n\n', cfg.tau);

% check the equilibrium and matrix dimensions
dx_eq = model.dynamics(model.x_eq, model.u_eq, model.d_eq);
assert(all(abs(dx_eq) < 1e-10), 'Equilibrium derivatives must be zero.');
assert(isequal(size(model.A), [2, 2]), 'A must be 2x2.');
assert(isequal(size(model.B), [2, 1]), 'B must be 2x1.');
assert(isequal(size(model.C), [1, 2]), 'C must be 1x2.');
assert(isscalar(model.D), 'D must be scalar.');

% compare transfer function coefficients using a monic denominator
drag_slope = cfg.b + 2*cfg.c*abs(cfg.v_eq);
expected_den = conv([cfg.m, drag_slope], [cfg.tau, 1]);
expected_num = 1/expected_den(1);
expected_den = expected_den/expected_den(1);
[num, den] = tfdata(model.tf, 'v');
num = num/den(1);
den = den/den(1);
assert(numel(den) == numel(expected_den), 'Unexpected denominator order.');
assert(numel(num) <= numel(expected_den), 'Unexpected numerator order.');
% allow leading zero coefficients and conversion roundoff
num = [zeros(1, numel(expected_den) - numel(num)), num];
expected_num = [0, 0, expected_num];
assert(all(abs(den - expected_den) < 1e-9*max(1, norm(expected_den, inf))), ...
    'Transfer function denominator does not match the derived model.');
assert(all(abs(num - expected_num) < 1e-9*max(1, norm(expected_num, inf))), ...
    'Transfer function numerator does not match the derived model.');

% display the equilibrium and deviation model
fprintf('Cruise equilibrium: %.2f m/s, %.2f N actuator/commanded force\n\n', ...
    model.x_eq(1), model.u_eq);
fprintf('State-space matrices (deviation model)\n');
fprintf('A = [%.6e  %.6e;\n     %.6e  %.6e]\n', model.A.');
fprintf('B = [%.6e; %.6e]\n', model.B);
fprintf('C = [%g  %g]\n', model.C);
fprintf('D = %g\n\n', model.D);

% display the transfer function with physical coefficients
v_eq = model.x_eq(1);
den = [cfg.m*cfg.tau, ...
       cfg.m + cfg.tau*(cfg.b + 2*cfg.c*abs(v_eq)), ...
       cfg.b + 2*cfg.c*abs(v_eq)];
fprintf('G(s) = 1 / (%.6g*s^2 %+.6g*s %+.6g)\n', den(1), den(2), den(3));

% check the 2D dynamics at straight cruising motion
x_2d = [cfg.X0; cfg.Y0; 0; model.x_eq];
dx_2d = vehicle_2d_model(cfg, x_2d, model.u_eq, 0, model.d_eq);
assert(isequal(size(dx_2d), [5, 1]), '2D derivatives must be 5x1.');
assert(abs(dx_2d(3)) < 1e-10, 'Zero steering must give zero heading rate.');
assert(abs(dx_2d(1) - x_2d(4)) < 1e-10 && abs(dx_2d(2)) < 1e-10, ...
    'At zero heading, velocity must be along the X axis.');
assert(all(abs(dx_2d(4:5) - dx_eq) < 1e-10), ...
    '2D longitudinal dynamics must match the cruise subsystem.');

% check reference paths and the original cruise matrices
paths = reference_path(cfg);
assert(all(paths.straight.Y == 0), 'Straight path must have zero Y coordinates.');
assert(all(isfinite([paths.curved.X, paths.curved.Y, paths.curved.psi])), ...
    'Curved path coordinates and headings must be finite.');
expected_A = [-drag_slope/cfg.m, 1/cfg.m; 0, -1/cfg.tau];
assert(norm(model.A - expected_A, inf) < 1e-10, 'Cruise A matrix changed.');
assert(norm(model.B - [0; 1/cfg.tau], inf) < 1e-10, 'Cruise B matrix changed.');
assert(isequal(model.C, [1, 0]) && model.D == 0, 'Cruise output matrices changed.');
assert(norm(model.E - [-1/cfg.m; 0], inf) < 1e-10, 'Cruise disturbance matrix changed.');

% display the 2D configuration
fprintf('\n2D bicycle model: %d states, wheelbase %.2f m, steering limit %.2f deg\n', ...
    numel(x_2d), cfg.L, rad2deg(cfg.steering_max));
fprintf('Initial pose: (%.2f, %.2f) m, heading %.2f rad\n', cfg.X0, cfg.Y0, cfg.psi0);

%% Stage 2 - system analysis
analysis = system_analysis(model, cfg);

% generate the Stage 1 open-loop figures
vehicle_2d_demo;

%% Stage 3 - controller design
stage3 = stage3_simulations(cfg, model);
