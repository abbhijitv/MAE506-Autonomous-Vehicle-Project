function results = stage3_simulations(cfg, model)
if nargin < 1
    cfg = config();
end
if nargin < 2
    model = vehicle_model(cfg);
end

% all controllers use the same nonlinear plant and force limits
controller = state_feedback(cfg, model);
observer = observer_controller(cfg, model);
results.feedback = controller;
results.pid = simulate_cruise(cfg, model, [], false);
results.pid_disturbance = simulate_cruise(cfg, model, [], true);
results.state_feedback = simulate_cruise(cfg, model, controller, false);
results.observer = simulate_observer(cfg, model, observer);
paths = reference_path(cfg);
results.path_straight = simulate_path(cfg, paths.straight);
results.path_curved = simulate_path(cfg, paths.curved);

figure_dir = fullfile(fileparts(mfilename('fullpath')), 'figures', 'stage3');
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end
results.plot_files = {fullfile(figure_dir, 'pid_speed_tracking.png'), ...
    fullfile(figure_dir, 'pid_disturbance_rejection.png'), ...
    fullfile(figure_dir, 'state_feedback_tracking.png'), ...
    fullfile(figure_dir, 'observer_estimation.png'), ...
    fullfile(figure_dir, 'path_tracking.png'), ...
    fullfile(figure_dir, 'steering_angle.png')};

% PID speed tracking
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
ax = axes('Parent', fig);
plot(ax, results.pid.t, results.pid.reference, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.pid.t, results.pid.x(:, 1), 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Speed (m/s)');
title(ax, 'PID cruise speed tracking');
legend(ax, 'Desired speed', 'Actual speed', 'Location', 'best');
grid(ax, 'on');
save_plot(fig, ax, results.plot_files{1});

% PID rejection of the configured uphill disturbance
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
ax = axes('Parent', fig);
plot(ax, results.pid_disturbance.t, results.pid_disturbance.reference, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.pid_disturbance.t, results.pid_disturbance.x(:, 1), 'LineWidth', 1.5);
xline(ax, cfg.disturbance_start, ':', 'Uphill disturbance', 'HandleVisibility', 'off');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Speed (m/s)');
title(ax, sprintf('PID disturbance rejection: %g N opposing force', cfg.disturbance_force));
legend(ax, 'Desired speed', 'Actual speed', 'Location', 'best');
grid(ax, 'on');
save_plot(fig, ax, results.plot_files{2});

% state-feedback speed and commanded force
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
layout = tiledlayout(fig, 2, 1);
ax = nexttile(layout);
plot(ax, results.state_feedback.t, results.state_feedback.reference, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.state_feedback.t, results.state_feedback.x(:, 1), 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Speed (m/s)');
title(ax, 'Integral state-feedback speed tracking');
legend(ax, 'Desired speed', 'Actual speed', 'Location', 'best');
grid(ax, 'on');
ax = nexttile(layout);
plot(ax, results.state_feedback.t, results.state_feedback.u, 'LineWidth', 1.5);
hold(ax, 'on');
yline(ax, cfg.F_max, '--'); yline(ax, cfg.F_min, '--');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Commanded force (N)');
legend(ax, 'Commanded force', 'Driving limit', 'Braking limit', 'Location', 'best');
grid(ax, 'on');
save_plot(fig, layout, results.plot_files{3});

% observer test uses a near-equilibrium plant with no disturbance increment
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
layout = tiledlayout(fig, 2, 1);
ax = nexttile(layout);
plot(ax, results.observer.t, results.observer.x(:, 1), 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.observer.t, results.observer.estimate(:, 1), '--', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Speed (m/s)');
title(ax, 'Luenberger observer near cruise equilibrium');
legend(ax, 'Actual speed', 'Estimated speed', 'Location', 'best');
grid(ax, 'on');
ax = nexttile(layout);
plot(ax, results.observer.t, results.observer.x(:, 2), 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.observer.t, results.observer.estimate(:, 2), '--', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Actuator force (N)');
legend(ax, 'Actual force', 'Estimated force', 'Location', 'best');
grid(ax, 'on');
save_plot(fig, layout, results.plot_files{4});

% curved-road trajectory and steering
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
ax = axes('Parent', fig);
plot(ax, paths.curved.X, paths.curved.Y, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.path_curved.x(:, 1), results.path_curved.x(:, 2), 'LineWidth', 1.5);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
title(ax, sprintf('Pure-pursuit path tracking at %g m/s', cfg.path_speed));
legend(ax, 'Reference path', 'Vehicle trajectory', 'Location', 'best');
% show road curvature without changing the trajectory data
path_X = [paths.curved.X(:); results.path_curved.x(:, 1)];
path_Y = [paths.curved.Y(:); results.path_curved.x(:, 2)];
xlim(ax, [min(path_X), max(path_X)]);
ylim(ax, [min(-10, min(path_Y) - 1), max(10, max(path_Y) + 1)]);
pbaspect(ax, [3, 1, 1]);
subtitle(ax, 'Vertical scale exaggerated for readability');
grid(ax, 'on');
save_plot(fig, ax, results.plot_files{5});

fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
ax = axes('Parent', fig);
plot(ax, results.path_curved.t, rad2deg(results.path_curved.steering), 'LineWidth', 1.5);
hold(ax, 'on');
yline(ax, rad2deg(cfg.steering_max), '--');
yline(ax, -rad2deg(cfg.steering_max), '--');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Steering angle (deg)');
title(ax, 'Pure-pursuit steering command');
legend(ax, 'Steering angle', 'Upper limit', 'Lower limit', 'Location', 'best');
ylim(ax, [-35 35]);
grid(ax, 'on');
save_plot(fig, ax, results.plot_files{6});

fprintf('\nStage 3 - Controller Design\n');
fprintf('PID gains: Kp = %g, Ki = %g, Kd = %g\n', cfg.pid_Kp, cfg.pid_Ki, cfg.pid_Kd);
fprintf('PID final speed error: %.4f m/s; with disturbance: %.4f m/s\n', ...
    cfg.v_ref - results.pid.x(end, 1), cfg.v_ref - results.pid_disturbance.x(end, 1));
fprintf('State-feedback Kx = [%.6g %.6g], Ki = %.6g; final error: %.4f m/s\n', ...
    controller.Kx, controller.Ki, cfg.v_ref - results.state_feedback.x(end, 1));
fprintf('Observer L = [%.6g; %.6g]; final errors: %.3e m/s, %.3e N\n', ...
    observer.L, results.observer.error(end, :));
fprintf('Pure pursuit: lookahead %.2f m, speed %.2f m/s, lateral RMS error %.4f m\n', ...
    cfg.path_lookahead, cfg.path_speed, sqrt(mean(results.path_curved.lateral_error.^2)));
end

function result = simulate_cruise(cfg, model, controller, disturbed)
t = cfg.t(:);
n = numel(t);
result.t = t;
result.reference = cfg.v_ref*ones(n, 1);
result.x = zeros(n, 2);
result.x(1, :) = [cfg.v0, cfg.Fa0];
result.u = zeros(n, 1);
result.integral = zeros(n, 1);
result.d = model.d_eq*ones(n, 1);
if disturbed
    result.d = result.d + cfg.disturbance_force*double(t >= cfg.disturbance_start);
end
memory = struct('integral', 0, 'derivative', 0, 'last_speed', cfg.v0);
z = 0;
for i = 1:n
    x = result.x(i, :)';
    e = cfg.v_ref - x(1);
    if isempty(controller)
        result.integral(i) = memory.integral;
        [u, memory] = pid_controller(cfg, x(1), memory);
    else
        result.integral(i) = z;
        raw = controller.command(x, z);
        u = min(max(raw, cfg.F_min), cfg.F_max);
        if ~((raw >= cfg.F_max && e > 0) || (raw <= cfg.F_min && e < 0))
            z = z + cfg.dt*e;
        end
    end
    result.u(i) = u;
    if i < n
        result.x(i + 1, :) = rk4_step(@(x) model.dynamics(x, u, result.d(i)), x, cfg.dt)';
    end
end
assert(all(isfinite([result.x(:); result.u; result.integral])), 'Cruise simulation must be finite.');
end

function result = simulate_observer(cfg, model, observer)
t = (0:cfg.dt:cfg.observer_duration)';
x0 = model.x_eq + [cfg.observer_speed_offset; 0];
z = zeros(numel(t), 4);
z(1, :) = [x0; x0 + cfg.observer_initial_error]';
for i = 1:numel(t) - 1
    dynamics = @(z) [model.dynamics(z(1:2), model.u_eq, model.d_eq); ...
        observer.dynamics(z(3:4), z(1), model.u_eq)];
    z(i + 1, :) = rk4_step(dynamics, z(i, :)', cfg.dt)';
end
result = observer;
result.t = t;
result.x = z(:, 1:2);
result.estimate = z(:, 3:4);
result.error = result.estimate - result.x;
assert(all(isfinite(z(:))), 'Observer simulation must be finite.');
end

function result = simulate_path(cfg, path)
assert(cfg.path_speed > 0 && cfg.path_lookahead > 0 ...
    && cfg.path_length > 2*cfg.path_lookahead, 'Invalid pure-pursuit settings.');
path.s = [0, cumsum(hypot(diff(path.X), diff(path.Y)))];
t = (0:cfg.dt:cfg.path_max_time)';
x = zeros(numel(t), 5);
steering = zeros(size(t));
lateral_error = zeros(size(t));
% equilibrium force keeps speed constant without overriding plant states
force = cfg.b*cfg.path_speed + cfg.c*cfg.path_speed*abs(cfg.path_speed) + cfg.d_eq;
assert(force >= cfg.F_min && force <= cfg.F_max, 'Path speed requires unavailable force.');
x(1, :) = [path.X(1), path.Y(1) + cfg.path_lateral_offset, path.psi(1), cfg.path_speed, force];
for i = 1:numel(t)
    [steering(i), lateral_error(i)] = path_tracking_controller(cfg, x(i, :)', path);
    % stop before the endpoint so the pursuit target stays ahead
    if x(i, 1) >= path.X(end) - cfg.path_lookahead
        break;
    end
    if i < numel(t)
        x(i + 1, :) = rk4_step(@(x) vehicle_2d_model(cfg, x, force, ...
            steering(i), cfg.d_eq), x(i, :)', cfg.dt)';
    end
end
result.t = t(1:i);
result.x = x(1:i, :);
result.steering = steering(1:i);
result.lateral_error = lateral_error(1:i);
assert(all(isfinite([result.x(:); result.steering; result.lateral_error])), ...
    'Path simulation must be finite.');
end

function next = rk4_step(dynamics, x, dt)
% commands are held constant over one controller sample
k1 = dynamics(x);
k2 = dynamics(x + dt*k1/2);
k3 = dynamics(x + dt*k2/2);
k4 = dynamics(x + dt*k3);
next = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function save_plot(fig, target, filename)
drawnow nocallbacks;
assert(isgraphics(fig, 'figure'), 'Stage 3 figure was closed before export.');
exportgraphics(target, filename, 'Resolution', 300);
close(fig);
end
