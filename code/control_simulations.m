function [results, controller] = control_simulations(cfg, model)
results.run_started = now;
A = model.A;
B = model.B;
C = model.C;
D = model.D;

% Desired performance specifications
Settling_Time = cfg.feedback_settling_time;
Overshoot = cfg.feedback_overshoot;
OS = Overshoot/100;
zeta = -log(OS)/sqrt(pi^2 + log(OS)^2);
wn = 4/(zeta*Settling_Time);

% Calculate desired poles
desiredPoles = [-zeta*wn + 1i*wn*sqrt(1-zeta^2), ...
                -zeta*wn - 1i*wn*sqrt(1-zeta^2)];

% Calculate state feedback gains
K = place(A,B,desiredPoles);

% Closed-loop A matrix
Acl = A-B*K;

% Linear systems
OL = ss(A,B,C,D);

OpenLoop_Gain = dcgain(OL);
% Unity reference tracking
Gain = -1/(C*(Acl\B));
CL = ss(Acl,B*Gain,C,D);

controller.K = K;
controller.Gain = Gain;
controller.Acl = Acl;
controller.desiredPoles = desiredPoles;
controller.poles = eig(Acl);
controller.OpenLoop_Gain = OpenLoop_Gain;
controller.OL = OL;
controller.CL = CL;
assert(abs(dcgain(CL) - 1) < 1e-10, 'Reference gain must give unity DC tracking.');

% common simulation conditions
results.pid = simulate_cruise(cfg, model, controller, 'pid');
results.state_feedback = simulate_cruise(cfg, model, controller, 'state_feedback');
results.open_loop = simulate_cruise(cfg, model, controller, 'open_loop');
results.pid.metrics = performance_metrics(cfg, results.pid);
results.state_feedback.metrics = performance_metrics(cfg, results.state_feedback);

figure_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end
results.plot_files = {fullfile(figure_dir, '01_pid_speed_tracking.png'), ...
    fullfile(figure_dir, '02_state_feedback_speed_tracking.png'), ...
    fullfile(figure_dir, '03_pid_vs_state_feedback.png'), ...
    fullfile(figure_dir, '04_open_loop_vs_closed_loop.png'), ...
    fullfile(figure_dir, '05_control_effort.png')};
results.figures = gobjects(1, 5);

% close previous comparison figures
previous_figures = findall(groot, 'Type', 'figure', 'Tag', 'MAE506Comparison');
for i = 1:numel(previous_figures)
    close(previous_figures(i));
end

% PID speed tracking
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off', 'Tag', 'MAE506Comparison');
ax = axes('Parent', fig);
plot(ax, results.pid.t, results.pid.reference, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.pid.t, results.pid.x(:, 1), 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Vehicle speed (m/s)');
title(ax, 'PID cruise speed tracking');
legend(ax, 'Desired speed', 'Actual speed', 'Location', 'best');
grid(ax, 'on');
results.figures(1) = fig;
save_plot(fig, ax, results.plot_files{1});

% state-feedback speed tracking
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off', 'Tag', 'MAE506Comparison');
ax = axes('Parent', fig);
plot(ax, results.state_feedback.t, results.state_feedback.reference, '--', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.state_feedback.t, results.state_feedback.x(:, 1), 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Vehicle speed (m/s)');
title(ax, 'Pole-placement state-feedback speed tracking');
legend(ax, 'Desired speed', 'Actual speed', 'Location', 'best');
grid(ax, 'on');
results.figures(2) = fig;
save_plot(fig, ax, results.plot_files{2});

% controller speed comparison
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off', 'Tag', 'MAE506Comparison');
ax = axes('Parent', fig);
plot(ax, results.pid.t, results.pid.reference, ':', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.pid.t, results.pid.x(:, 1), 'LineWidth', 1.5);
plot(ax, results.state_feedback.t, results.state_feedback.x(:, 1), '--', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Vehicle speed (m/s)');
title(ax, 'PID vs. state-feedback speed tracking');
legend(ax, 'Desired speed', 'PID', 'State feedback', 'Location', 'best');
grid(ax, 'on');
results.figures(3) = fig;
save_plot(fig, ax, results.plot_files{3});

% open-loop comparison
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off', 'Tag', 'MAE506Comparison');
ax = axes('Parent', fig);
plot(ax, results.open_loop.t, results.open_loop.x(:, 1), 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.state_feedback.t, results.state_feedback.x(:, 1), '--', 'LineWidth', 1.5);
plot(ax, results.open_loop.t, results.open_loop.reference, ':', 'LineWidth', 1.5);
xlabel(ax, 'Time (s)'); ylabel(ax, 'Vehicle speed (m/s)');
title(ax, 'Open-loop vs. state-feedback closed-loop response');
legend(ax, 'Open loop', 'State feedback', 'Desired speed', 'Location', 'best');
grid(ax, 'on');
results.figures(4) = fig;
save_plot(fig, ax, results.plot_files{4});

% commanded force comparison
fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off', 'Tag', 'MAE506Comparison');
ax = axes('Parent', fig);
plot(ax, results.pid.t, results.pid.u, 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, results.state_feedback.t, results.state_feedback.u, '--', 'LineWidth', 1.5);
yline(ax, cfg.F_max, ':'); yline(ax, cfg.F_min, ':');
xlabel(ax, 'Time (s)'); ylabel(ax, 'Commanded force (N)');
title(ax, 'PID vs. state-feedback control effort');
legend(ax, 'PID command', 'State-feedback command', 'Driving limit', 'Braking limit', 'Location', 'best');
grid(ax, 'on');
results.figures(5) = fig;
save_plot(fig, ax, results.plot_files{5});

% controller performance metrics
fprintf('\nController Comparison\n');
fprintf('PID: Kp = %g, Ki = %g, Kd = %g\n', cfg.pid_Kp, cfg.pid_Ki, cfg.pid_Kd);
fprintf('State feedback: K = [%.6g %.6g], Gain = %.6g\n', controller.K, controller.Gain);
fprintf('Closed-loop poles (1/s)\n'); disp(controller.poles);
fprintf('%-16s %12s %14s %14s %16s %14s\n', ...
    'Controller', 'Rise (s)', 'Settling (s)', 'Overshoot (%)', 'SS error (m/s)', 'Peak |u| (N)');
names = {'PID', 'State feedback'};
metrics = {results.pid.metrics, results.state_feedback.metrics};
for i = 1:2
    metric = metrics{i};
    fprintf('%-16s %12.3f %14.3f %14.3f %16.6e %14.3f\n', names{i}, ...
        metric.rise_time, metric.settling_time, metric.overshoot, ...
        metric.steady_state_error, metric.peak_control);
end
end

function result = simulate_cruise(cfg, model, controller, method)
t = cfg.t(:);
n = numel(t);
result.t = t;
result.reference = cfg.v_ref*ones(n, 1);
result.x = zeros(n, 2);
result.x(1, :) = [cfg.v0, cfg.Fa0];
result.u = zeros(n, 1);
memory = struct('integral', 0, 'derivative', 0, 'last_speed', cfg.v0);
if strcmp(method, 'pid')
    result.integral = zeros(n, 1);
end
feedforward = cfg.b*cfg.v_ref + cfg.c*cfg.v_ref*abs(cfg.v_ref) + cfg.d_eq;
for i = 1:n
    x = result.x(i, :)';
    if strcmp(method, 'pid')
        result.integral(i) = memory.integral;
        [u, memory] = pid_controller(cfg, x(1), memory);
    elseif strcmp(method, 'state_feedback')
        raw = model.u_eq - controller.K*(x - model.x_eq) ...
            + controller.Gain*(cfg.v_ref - model.x_eq(1));
        u = min(max(raw, cfg.F_min), cfg.F_max);
    else
        u = min(max(feedforward, cfg.F_min), cfg.F_max);
    end
    result.u(i) = u;
    if i < n
        result.x(i + 1, :) = rk4_step(@(x) model.dynamics(x, u, cfg.d_eq), x, cfg.dt)';
    end
end
assert(all(isfinite([result.x(:); result.u])), 'Cruise simulation must be finite.');
end

function metric = performance_metrics(cfg, result)
% common tracking definitions
step_size = cfg.v_ref - cfg.v0;
assert(step_size ~= 0, 'Tracking metrics require a nonzero speed change.');
response = (result.x(:, 1) - cfg.v0)/step_size;
low = find(response >= cfg.rise_limits(1), 1);
high = find(response >= cfg.rise_limits(2), 1);
metric.rise_time = NaN;
if ~isempty(low) && ~isempty(high)
    metric.rise_time = result.t(high) - result.t(low);
end
outside = find(abs(result.x(:, 1) - cfg.v_ref) > ...
    cfg.settling_threshold*abs(step_size), 1, 'last');
metric.settling_time = NaN;
if isempty(outside)
    metric.settling_time = result.t(1);
elseif outside < numel(result.t)
    metric.settling_time = result.t(outside + 1);
end
metric.overshoot = max(0, 100*(max(response) - 1));
tail = result.t >= result.t(end) - cfg.steady_state_window;
metric.steady_state_error = cfg.v_ref - mean(result.x(tail, 1));
metric.peak_control = max(abs(result.u));
end

function next = rk4_step(dynamics, x, dt)
% commands held constant
k1 = dynamics(x);
k2 = dynamics(x + dt*k1/2);
k3 = dynamics(x + dt*k2/2);
k4 = dynamics(x + dt*k3);
next = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function save_plot(fig, target, filename)
drawnow nocallbacks;
assert(isgraphics(fig, 'figure'), 'Figure was closed before export.');
exportgraphics(target, filename, 'Resolution', 300);
if batchStartupOptionUsed
    close(fig);
else
    set(fig, 'HandleVisibility', 'on', 'Visible', 'on');
end
end
