clc;
clear;

% load the configuration and create the vehicle model
cfg = config();
model = vehicle_model(cfg);

% display the vehicle parameters
fprintf('Autonomous Vehicle Cruise Control\n');
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

% analyze the vehicle
analysis = system_analysis(model, cfg);
[results, controller] = control_simulations(cfg, model);

% save numerical results
results.report_file = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'simulation_results.txt');
save_results(results.report_file, cfg, model, analysis, controller, results);
saved_files = dir(fileparts(results.report_file));
saved_files = saved_files(~[saved_files.isdir]);
saved_files = saved_files(~strcmp({saved_files.name}, '06_linearized_state_feedback.png'));
assert(numel(saved_files) == 6 && all(cellfun(@isfile, [results.plot_files, {results.report_file}])), ...
    'Results must contain exactly five PNGs and simulation_results.txt.');

function save_results(filename, cfg, model, analysis, controller, results)
[fid, message] = fopen(filename, 'w');
assert(fid ~= -1, 'Cannot open numerical results: %s', message);
file_cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Autonomous Vehicle Cruise Control\n');
fprintf(fid, '\n');
fprintf(fid, 'Vehicle Parameters\n');
fprintf(fid, 'Mass: %.6f kg\n', cfg.m);
fprintf(fid, 'Linear resistance: %.6f N*s/m\n', cfg.b);
fprintf(fid, 'Quadratic drag: %.6f N*s^2/m^2\n', cfg.c);
fprintf(fid, 'Actuator time constant: %.6f s\n', cfg.tau);
fprintf(fid, 'Initial speed: %.6f m/s\n', cfg.v0);
fprintf(fid, 'Initial actuator force: %.6f N\n', cfg.Fa0);
fprintf(fid, 'Reference speed: %.6f m/s\n', cfg.v_ref);
fprintf(fid, 'Duration: %.6f s\n', cfg.t_end);
fprintf(fid, 'Time step: %.6f s\n', cfg.dt);
fprintf(fid, 'Command limits: %.6f to %.6f N\n', cfg.F_min, cfg.F_max);

% equilibrium and model
v_eq = model.x_eq(1);
drag_force = cfg.b*v_eq + cfg.c*v_eq*abs(v_eq);
fprintf(fid, '\nCruise Equilibrium\n');
fprintf(fid, 'Equilibrium speed: %.6f m/s\n', v_eq);
fprintf(fid, 'Equilibrium drag force: %.6f N\n', drag_force);
fprintf(fid, 'Equilibrium actuator force: %.6f N\n', model.x_eq(2));
fprintf(fid, 'Equilibrium command: %.6f N\n', model.u_eq);
fprintf(fid, 'Equilibrium opposing force: %.6f N\n', model.d_eq);
fprintf(fid, '\nState-Space Model\n');
fprintf(fid, 'States: speed (m/s), actual actuator force (N).\n');
fprintf(fid, 'Input: commanded actuator force (N). Output: speed (m/s).\n');
fprintf(fid, 'Linear matrices describe deviations from cruise equilibrium.\n');
write_values(fid, 'A', model.A);
write_values(fid, 'B', model.B);
write_values(fid, 'C', model.C);
write_values(fid, 'D', model.D);
write_values(fid, 'Optional disturbance matrix E', model.E);

[num, den] = tfdata(model.tf, 'v');
fprintf(fid, '\nTransfer Function\n');
fprintf(fid, 'Command-force deviation to speed deviation; descending powers of s.\n');
write_values(fid, 'Numerator', num);
write_values(fid, 'Denominator', den);
scale = cfg.m*cfg.tau/den(1);
write_values(fid, 'Physical numerator', num*scale);
write_values(fid, 'Physical denominator', den*scale);

fprintf(fid, '\nOpen-Loop Analysis\n');
write_values(fid, 'Open-loop eigenvalues (1/s)', analysis.poles);
fprintf(fid, 'Open-loop stability: %s\n', stability_text(analysis.poles));
write_values(fid, 'Controllability matrix', analysis.controllability);
fprintf(fid, 'Controllability rank: %d/%d\n', analysis.controllability_rank, size(model.A, 1));
write_values(fid, 'Observability matrix', analysis.observability);
fprintf(fid, 'Observability rank: %d/%d\n', analysis.observability_rank, size(model.A, 1));
if ~isempty(analysis.Wc)
    write_values(fid, 'Controllability Gramian', analysis.Wc);
    write_values(fid, 'Observability Gramian', analysis.Wo);
    write_values(fid, 'Controllability Gramian eigenvalues', analysis.Wc_eigenvalues);
    write_values(fid, 'Observability Gramian eigenvalues', analysis.Wo_eigenvalues);
    write_values(fid, 'Lyapunov residuals', analysis.gramian_residuals);
end
fprintf(fid, 'Controllable-form similarity error: %.9e\n', analysis.controllable_form.similarity_error);
fprintf(fid, 'Controllable-form transfer-function error: %.9e\n', analysis.controllable_form.tf_error);
fprintf(fid, 'Observable-form similarity error: %.9e\n', analysis.observable_form.similarity_error);
fprintf(fid, 'Observable-form transfer-function error: %.9e\n', analysis.observable_form.tf_error);

fprintf(fid, '\nPID Controller\n');
fprintf(fid, 'Kp: %.6f N/(m/s)\n', cfg.pid_Kp);
fprintf(fid, 'Ki: %.6f N/m\n', cfg.pid_Ki);
fprintf(fid, 'Kd: %.6f N/(m/s^2)\n', cfg.pid_Kd);
fprintf(fid, 'Derivative filter time constant: %.6f s\n', cfg.pid_filter_tau);
fprintf(fid, '\nState-Feedback Controller\n');
fprintf(fid, 'Settling-time specification: %.6f s\n', cfg.feedback_settling_time);
fprintf(fid, 'Overshoot specification: %.6f %%\n', cfg.feedback_overshoot);
write_values(fid, 'Desired state-feedback poles (1/s)', controller.desiredPoles);
write_values(fid, 'State-feedback gain K', controller.K);
fprintf(fid, 'Reference gain: %.9e N/(m/s)\n', controller.Gain);
fprintf(fid, 'Reference gain applies to the comparison only.\n');
write_values(fid, 'Closed-loop eigenvalues (1/s)', controller.poles);
fprintf(fid, 'Closed-loop stability: %s (linearized, unsaturated)\n', stability_text(controller.poles));

% measured performance
names = {'PID', 'State feedback'};
metrics = {results.pid.metrics, results.state_feedback.metrics};
for i = 1:2
    metric = metrics{i};
    fprintf(fid, '\n%s Performance\n', names{i});
    fprintf(fid, 'Rise time: %.6f s\n', metric.rise_time);
    fprintf(fid, 'Settling time: %.6f s\n', metric.settling_time);
    fprintf(fid, 'Overshoot: %.6f %%\n', metric.overshoot);
    fprintf(fid, 'Steady-state error: %.9e m/s\n', metric.steady_state_error);
    fprintf(fid, 'Peak actuator command: %.6f N\n', metric.peak_control);
end
fprintf(fid, '\nPerformance Comparison\n');
fprintf(fid, '%-16s %12s %14s %14s %20s %18s\n', ...
    'Controller', 'Rise (s)', 'Settling (s)', 'Overshoot (%)', 'SS error (m/s)', 'Peak command (N)');
for i = 1:2
    metric = metrics{i};
    fprintf(fid, '%-16s %12.6f %14.6f %14.6f %20.9e %18.6f\n', names{i}, ...
        metric.rise_time, metric.settling_time, metric.overshoot, ...
        metric.steady_state_error, metric.peak_control);
end
fprintf(fid, '\nMetric Definitions\n');
fprintf(fid, 'Rise time: first sampled %.1f%% to %.1f%% crossings of the requested speed change.\n', ...
    100*cfg.rise_limits);
fprintf(fid, 'Settling time: last exit from the %.1f%% tracking band, followed by one sample.\n', ...
    100*cfg.settling_threshold);
fprintf(fid, 'Steady-state error: target minus mean speed over the final %.6f s.\n', cfg.steady_state_window);
fprintf(fid, 'Peak command uses the actual saturated command, not actuator-force state.\n');
fprintf(fid, 'Plot speeds are absolute vehicle speeds; the linear model uses deviations.\n');
fprintf(fid, 'State-feedback specifications describe the unsaturated linear design.\n');
fprintf(fid, 'NaN response times indicate an unreached threshold or an unsettled response.\n');
fprintf(fid, '\nLinearized State-Feedback Performance\n');
fprintf(fid, 'Design settling-time target: %.6f s\n', cfg.feedback_settling_time);
fprintf(fid, 'Design overshoot target: %.6f %%\n', cfg.feedback_overshoot);
fprintf(fid, 'Measured linearized settling time: %.6f s\n', results.linearized.settling_time);
fprintf(fid, 'Measured linearized overshoot: %.6f %%\n', results.linearized.overshoot);
[message, error_code] = ferror(fid);
assert(error_code == 0, 'Cannot write numerical results: %s', message);
clear file_cleanup;
end

function write_values(fid, name, values)
fprintf(fid, '%s:\n', name);
for row = 1:size(values, 1)
    for column = 1:size(values, 2)
        value = values(row, column);
        if isreal(value)
            fprintf(fid, '  %.9e', value);
        else
            fprintf(fid, '  %.9e%+.9ei', real(value), imag(value));
        end
    end
    fprintf(fid, '\n');
end
end

function text = stability_text(poles)
if all(real(poles) < 0)
    text = 'Asymptotically stable';
elseif any(real(poles) > 0)
    text = 'Unstable';
else
    text = 'Zero-real-part poles; not asymptotically stable, further analysis required';
end
end
