clc;
clear;

% load the configuration and create the vehicle model
cfg = config();
model = vehicle_model(cfg);

% display the vehicle parameters
fprintf('Vehicle parameters\n');
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
fprintf('Cruising equilibrium\n');
fprintf('Speed: %.2f m/s\n', model.x_eq(1));
fprintf('Actuator force: %.2f N\n', model.x_eq(2));
fprintf('Commanded force: %.2f N\n', model.u_eq);
fprintf('Disturbance force: %.2f N\n\n', model.d_eq);
fprintf('State-space matrices (deviation model)\n');
fprintf('A =\n'); disp(model.A);
fprintf('B =\n'); disp(model.B);
fprintf('C =\n'); disp(model.C);
fprintf('D =\n'); disp(model.D);
fprintf('E =\n'); disp(model.E);

% display the transfer function with physical coefficients
v_eq = model.x_eq(1);
den = [cfg.m*cfg.tau, ...
       cfg.m + cfg.tau*(cfg.b + 2*cfg.c*abs(v_eq)), ...
       cfg.b + 2*cfg.c*abs(v_eq)];
fprintf('Transfer function\n');
fprintf('G(s) = 1 / (%.6g*s^2 %+.6g*s %+.6g)\n', den(1), den(2), den(3));
