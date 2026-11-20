% run the complete project
addpath(fileparts(fileparts(mfilename('fullpath'))));
main;

% vehicle model checks
expected_A = [-(cfg.b + 2*cfg.c*abs(cfg.v_eq))/cfg.m, 1/cfg.m; 0, -1/cfg.tau];
assert(norm(model.A - expected_A, inf) < 1e-12, 'Unexpected vehicle A matrix.');
assert(isequal(model.B, [0; 1/cfg.tau]) && isequal(model.C, [1, 0]) && model.D == 0, ...
    'Unexpected vehicle input/output matrices.');
assert(all(abs(model.dynamics(model.x_eq, model.u_eq, cfg.d_eq)) < 1e-10), ...
    'Cruise equilibrium is inconsistent.');
assert(analysis.stable, 'Default vehicle must be asymptotically stable.');
assert(analysis.controllability_rank == 2 && analysis.observability_rank == 2, ...
    'Default vehicle must be controllable and observable.');
expected_poles = sort([expected_A(1, 1); expected_A(2, 2)]);
assert(norm(sort(analysis.poles) - expected_poles, inf) < 1e-10, 'Unexpected open-loop poles.');
assert(all(analysis.gramian_residuals < 1e-9), 'Lyapunov equations must hold.');
for s = [0, 1, 1i]
    expected = 1/((cfg.m*s + cfg.b + 2*cfg.c*abs(cfg.v_eq))*(cfg.tau*s + 1));
    assert(abs(evalfr(model.tf, s) - expected) < 1e-10, 'Transfer function is inconsistent.');
end

% Reference scaling
assert(abs(dcgain(controller.CL) - 1) < 1e-10, 'Reference gain must provide unity DC tracking.');
assert(all(real(controller.poles) < 0), 'Closed-loop poles must be stable.');
assert(max(abs(sort(controller.poles) - sort(controller.desiredPoles(:)))) < 1e-8, ...
    'Closed-loop poles must match the original design.');
assert(~isfield(controller, 'Ki'), 'State feedback must not include integral action.');
reference_delta = 1;
steady_delta = -model.C*(controller.Acl\(model.B*controller.Gain*reference_delta));
assert(abs(steady_delta - reference_delta) < 1e-10, 'Deviation-reference scaling is incorrect.');

% original design checks
assert(cfg.feedback_settling_time == 2 && cfg.feedback_overshoot == 7.5, ...
    'Original performance specifications must be preserved.');
OS = cfg.feedback_overshoot/100;
zeta = -log(OS)/sqrt(pi^2 + log(OS)^2);
wn = 4/(zeta*cfg.feedback_settling_time);
force_gain = cfg.tau*(2*zeta*wn + model.A(1, 1)) - 1;
speed_gain = cfg.m*cfg.tau*(wn^2 + model.A(1, 1)*(1 + force_gain)/cfg.tau);
assert(norm(controller.K - [speed_gain, force_gain], inf) < 1e-6, ...
    'Feedback gains differ from the original pole design.');
project_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
code_dir = fullfile(project_dir, 'code');
original_file = fullfile(code_dir, 'tests', 'fixtures', 'cruisecontrol_minimalcode4.original');
assert(isequal(fileread(fullfile(code_dir, 'cruisecontrol_minimalcode4.m')), fileread(original_file)), ...
    'Standalone script differs from its original reference source.');
assert(isequal(fileread(fullfile(code_dir, 'pid_controller.m')), ...
    fileread(fullfile(code_dir, 'tests', 'fixtures', 'pid_controller.original'))), ...
    'Original PID implementation must be preserved.');

% identical comparison conditions
assert(isequal(results.pid.t, results.state_feedback.t) ...
    && isequal(results.pid.t, cfg.t(:)), 'Controllers must share the same time grid.');
assert(isequal(results.pid.reference, results.state_feedback.reference) ...
    && all(results.pid.reference == cfg.v_ref), 'Controllers must share the physical reference.');
assert(isequal(results.pid.x(1, :), results.state_feedback.x(1, :)) ...
    && isequal(results.pid.x(1, :), [cfg.v0, cfg.Fa0]), 'Initial states must match.');
cruise_results = {results.pid, results.state_feedback};
for i = 1:numel(cruise_results)
    result = cruise_results{i};
    assert(all(isfinite([result.x(:); result.u])), 'Simulation contains nonfinite values.');
    assert(all(result.u >= cfg.F_min & result.u <= cfg.F_max), 'Commanded-force limit exceeded.');
    assert(all(result.x(:, 2) >= cfg.F_min - 1e-8 & result.x(:, 2) <= cfg.F_max + 1e-8), ...
        'Actuator force exceeds the physical limits.');
    tail = result.t >= cfg.t_end - cfg.steady_state_window;
    assert(max(abs(result.x(tail, 1) - cfg.v_ref)) < 0.05, 'Default tracking did not converge.');
    metric = result.metrics;
    assert(all(isfinite([metric.rise_time, metric.settling_time, metric.overshoot, ...
        metric.steady_state_error, metric.peak_control])), 'Default metrics must be finite.');
    assert(metric.rise_time >= 0 && metric.settling_time >= 0 && metric.overshoot >= 0, ...
        'Response-time and overshoot metrics must be nonnegative.');
    assert(abs(metric.peak_control - max(abs(result.u))) < 1e-10, 'Peak command metric is inconsistent.');
    assert(abs(metric.steady_state_error - (cfg.v_ref - mean(result.x(tail, 1)))) < 1e-10, ...
        'Steady-state metric is inconsistent.');
    % sampled nonlinear plant check
    for j = [1, floor(numel(result.t)/2)]
        x0 = result.x(j, :)';
        u = result.u(j);
        [~, check] = ode45(@(t, x) model.dynamics(x, u, cfg.d_eq), [0, cfg.dt], x0);
        assert(norm(check(end, :) - result.x(j + 1, :), inf) < 1e-5, ...
            'Simulation does not match the common nonlinear plant.');
    end
end
assert(all(isfinite([results.open_loop.x(:); results.open_loop.u])), 'Open-loop result must be finite.');

% PID anti-windup check
outward = (results.pid.u(1:end-1) >= cfg.F_max ...
    & cfg.v_ref - results.pid.x(1:end-1, 1) > 0) ...
    | (results.pid.u(1:end-1) <= cfg.F_min & cfg.v_ref - results.pid.x(1:end-1, 1) < 0);
integral_step = diff(results.pid.integral);
assert(any(outward) && all(abs(integral_step(outward)) < 1e-12), 'PID anti-windup failed.');

% figure export checks
expected_names = {'01_pid_speed_tracking.png', '02_state_feedback_speed_tracking.png', ...
    '03_pid_vs_state_feedback.png', '04_open_loop_vs_closed_loop.png', '05_control_effort.png'};
assert(numel(results.plot_files) == 5, 'Expected exactly five final plots.');
for i = 1:numel(expected_names)
    expected_path = fullfile(project_dir, 'results', expected_names{i});
    assert(strcmp(results.plot_files{i}, expected_path), 'Incorrect final plot path.');
    info = dir(expected_path);
    assert(~isempty(info) && info.bytes > 0, 'Project plot is missing.');
    image_info = imfinfo(expected_path);
    assert(strcmpi(image_info.Format, 'png'), 'Project plot must be PNG.');
    assert(info.datenum >= results.run_started - 2/86400, 'Plot was not updated during this run.');
end
saved = dir(fullfile(project_dir, 'results', '*.png'));
assert(isequal(sort({saved.name}), sort(expected_names)), 'Unexpected PNG files in results.');

% numerical report checks
report_path = fullfile(project_dir, 'results', 'simulation_results.txt');
assert(strcmp(results.report_file, report_path), 'Incorrect numerical report path.');
report = fileread(report_path);
assert(contains(report, ['Generated: ', datestr(results.run_started, 'yyyy-mm-dd HH:MM:SS.FFF')]), ...
    'Report does not belong to this simulation run.');
assert(contains(report, sprintf('Mass: %.6f kg', cfg.m)) ...
    && contains(report, sprintf('Equilibrium speed: %.6f m/s', model.x_eq(1))) ...
    && contains(report, sprintf('Kp: %.6f N/(m/s)', cfg.pid_Kp)) ...
    && contains(report, sprintf('Ki: %.6f N/m', cfg.pid_Ki)) ...
    && contains(report, sprintf('Kd: %.6f N/(m/s^2)', cfg.pid_Kd)) ...
    && contains(report, sprintf('Reference gain: %.9e N/(m/s)', controller.Gain)), ...
    'Report parameters or controller gains do not match this run.');
check_report_values(report, 'A', model.A);
check_report_values(report, 'B', model.B);
check_report_values(report, 'C', model.C);
check_report_values(report, 'D', model.D);
[num, den] = tfdata(model.tf, 'v');
check_report_values(report, 'Numerator', num);
check_report_values(report, 'Denominator', den);
check_report_values(report, 'Open-loop eigenvalues (1/s)', analysis.poles);
check_report_values(report, 'Controllability matrix', analysis.controllability);
check_report_values(report, 'Observability matrix', analysis.observability);
check_report_values(report, 'State-feedback gain K', controller.K);
assert(contains(report, sprintf('Controllability rank: %d/%d', analysis.controllability_rank, size(model.A, 1))) ...
    && contains(report, sprintf('Observability rank: %d/%d', analysis.observability_rank, size(model.A, 1))), ...
    'Report ranks do not match the analysis.');
assert(contains(report, 'Open-loop stability: Asymptotically stable') ...
    && contains(report, 'Closed-loop stability: Asymptotically stable'), 'Default stability report is incorrect.');
for pole = [controller.desiredPoles(:); controller.poles(:)].'
    assert(contains(report, sprintf('%.9e%+.9ei', real(pole), imag(pole))), ...
        'Report is missing a calculated controller pole.');
end
names = {'PID', 'State feedback'};
metrics = {results.pid.metrics, results.state_feedback.metrics};
for i = 1:2
    metric = metrics{i};
    assert(contains(report, [names{i}, ' Performance']), 'Individual performance section is missing.');
    row = sprintf('%-16s %12.6f %14.6f %14.6f %20.9e %18.6f', names{i}, ...
        metric.rise_time, metric.settling_time, metric.overshoot, ...
        metric.steady_state_error, metric.peak_control);
    assert(contains(report, row), 'Comparison table does not match the measured metrics.');
end
all_names = [expected_names, {'simulation_results.txt'}];
saved = dir(fullfile(project_dir, 'results'));
saved = saved(~[saved.isdir]);
assert(isequal(sort({saved.name}), sort(all_names)), 'Results folder must contain exactly six files.');
assert(all([saved.datenum] >= results.run_started - 2/86400), 'A result file was not updated this run.');
open_files = fopen('all');
for fid = open_files(:)'
    assert(~strcmp(fopen(fid), report_path), 'Numerical report was not closed.');
end
if batchStartupOptionUsed
    assert(~any(isgraphics(results.figures)), 'Batch figures must close after export.');
    assert(isempty(findall(groot, 'Type', 'figure')), 'Batch simulations must not leave figures open.');
else
    assert(all(isgraphics(results.figures, 'figure')), 'All five comparison figures must remain open.');
    for i = 1:5
        assert(strcmp(get(results.figures(i), 'Visible'), 'on'), 'Comparison figures must be visible.');
    end
    project_figures = findall(groot, 'Type', 'figure', 'Tag', 'MAE506Comparison');
    assert(numel(project_figures) == 5, 'Expected five visible comparison figures.');
end

function check_report_values(report, name, expected)
pattern = ['(?m)^', regexptranslate('escape', name), ':\r?\n((?:[^\r\n]+\r?\n){', ...
    num2str(size(expected, 1)), '})'];
match = regexp(report, pattern, 'tokens', 'once');
assert(~isempty(match), 'Report matrix or vector is missing.');
values = sscanf(match{1}, '%f');
assert(numel(values) == numel(expected), 'Report matrix dimensions are incorrect.');
values = reshape(values, size(expected, 2), size(expected, 1))';
assert(norm(values - expected, 'fro') < 1e-8*max(1, norm(expected, 'fro')), ...
    'Report matrix values do not match this run.');
end
