% main also runs the model checks
addpath(fileparts(fileparts(mfilename('fullpath'))));
main;
assert(size(x, 2) == 5 && all(isfinite(x(:))), 'Stage 1 open-loop states must be finite with five columns.');
assert(analysis.stable, 'Default cruise model must be asymptotically stable.');
assert(analysis.controllability_rank == 2 && analysis.observability_rank == 2, ...
    'Default cruise model must be controllable and observable.');
expected_poles = sort([-(cfg.b + 2*cfg.c*abs(cfg.v_eq))/cfg.m; -1/cfg.tau]);
assert(norm(sort(analysis.poles) - expected_poles, inf) < 1e-10, 'Unexpected cruise poles.');
assert(all(analysis.gramian_residuals < 1e-9), 'Lyapunov equations must hold.');

% compare simulated step responses with their analytic solutions
k = cfg.b + 2*cfg.c*abs(cfg.v_eq);
p = k/cfg.m;
q = 1/cfg.tau;
t = analysis.t;
if abs(q - p) < 1e-10*max(p, q)
    force_expected = cfg.v_eq + cfg.analysis_force_step/k*(1 - (1 + p*t).*exp(-p*t));
else
    force_expected = cfg.v_eq + cfg.analysis_force_step/k ...
        * (1 - (q*exp(-p*t) - p*exp(-q*t))/(q - p));
end
assert(max(abs(analysis.force_speed - force_expected)) < 1e-7, ...
    'Force step response does not match its analytic solution.');
elapsed = max(t - cfg.disturbance_start, 0);
disturbance_expected = cfg.v_eq - cfg.analysis_disturbance_step/k*(1 - exp(-p*elapsed));
assert(max(abs(analysis.disturbance_speed - disturbance_expected)) < 1e-7, ...
    'Disturbance response does not match its analytic solution.');
for j = 1:numel(analysis.plot_files)
    info = dir(analysis.plot_files{j});
    assert(~isempty(info) && info.bytes > 0, 'Stage 2 plot must be nonempty.');
end

% the same limits apply to all nonlinear cruise tests
cruise_results = {stage3.pid, stage3.pid_disturbance, stage3.state_feedback};
for j = 1:numel(cruise_results)
    result = cruise_results{j};
    assert(all(isfinite([result.x(:); result.u; result.integral])), 'Nonfinite cruise result.');
    assert(all(result.u <= cfg.F_max + 1e-9 & result.u >= cfg.F_min - 1e-9), ...
        'Commanded force exceeds the actuator limits.');
    % 0.05 m/s is 0.2 percent of the default cruising speed
    tail = result.t >= cfg.t_end - 5;
    assert(max(abs(result.x(tail, 1) - cfg.v_ref)) < 0.05, 'Cruise tracking has not converged.');
    assert(max(result.x(:, 1)) < 1.1*cfg.v_ref, 'Cruise overshoot exceeds 10 percent.');
    % conditional integration must freeze during outward saturation
    e = cfg.v_ref - result.x(1:end-1, 1);
    outward = (result.u(1:end-1) >= cfg.F_max & e > 0) ...
        | (result.u(1:end-1) <= cfg.F_min & e < 0);
    integral_step = diff(result.integral);
    assert(any(outward) && all(abs(integral_step(outward)) < 1e-12), ...
        'Integral anti-windup did not freeze during saturation.');
end
after_step = stage3.pid_disturbance.t >= cfg.disturbance_start;
assert(min(stage3.pid_disturbance.x(after_step, 1)) < cfg.v_ref - 0.1, ...
    'The disturbance must produce a visible speed reduction before recovery.');
assert(all(stage3.pid_disturbance.d(after_step) == cfg.d_eq + cfg.disturbance_force), ...
    'The uphill disturbance was not applied.');
assert(all(real(stage3.feedback.poles) < 0), 'Feedback poles must be stable.');
assert(all(real(stage3.observer.poles) < min(real(stage3.feedback.poles))), ...
    'Observer poles must be faster than the controller poles.');

% incorrect estimates should converge in the local, disturbance-free test
tail = stage3.observer.t >= cfg.observer_duration - 5;
assert(max(abs(stage3.observer.error(tail, 1))) < 1e-3, 'Observer speed error is too large.');
assert(max(abs(stage3.observer.error(tail, 2))) < 1, 'Observer force error is too large.');
assert(all(isfinite([stage3.observer.x(:); stage3.observer.estimate(:)])), ...
    'Observer results must be finite.');

% allow the first 30 m for correction of the initial lateral offset
path_results = {stage3.path_straight, stage3.path_curved};
for j = 1:numel(path_results)
    result = path_results{j};
    assert(all(isfinite([result.x(:); result.steering; result.lateral_error])), ...
        'Nonfinite path-tracking result.');
    assert(all(abs(result.steering) <= cfg.steering_max + 1e-12), 'Steering limit exceeded.');
    assert(all(diff(result.x(:, 1)) > 0), 'Vehicle must move forward along the road.');
    assert(result.x(end, 1) >= cfg.path_length - cfg.path_lookahead, ...
        'Vehicle stopped before covering the demonstration path.');
    tail = result.x(:, 1) >= 30;
    assert(any(tail), 'Path test must cover the convergence interval.');
    assert(sqrt(mean(result.lateral_error(tail).^2)) < 0.5 ...
        && max(abs(result.lateral_error(tail))) < 0.75, 'Path error exceeds the demonstration tolerance.');
    assert(max(abs(result.x(:, 4) - cfg.path_speed)) < 1e-8, 'Path speed equilibrium was not preserved.');
end

% all twelve plots must be saved, with no obsolete root PNGs or open figures
project_dir = fileparts(fileparts(mfilename('fullpath')));
stage1_files = {'vehicle_trajectory.png', 'vehicle_speed.png', 'vehicle_heading.png'};
for j = 1:numel(stage1_files)
    info = dir(fullfile(project_dir, 'figures', 'stage1', stage1_files{j}));
    assert(~isempty(info) && info.bytes > 0, 'Stage 1 plot is missing.');
end
for j = 1:numel(stage3.plot_files)
    info = dir(stage3.plot_files{j});
    assert(~isempty(info) && info.bytes > 0, 'Stage 3 plot is missing.');
end
assert(isempty(dir(fullfile(project_dir, 'figures', '*.png'))), 'Obsolete root PNGs remain.');
assert(isempty(findall(groot, 'Type', 'figure')), 'Simulations must not leave figures open.');
assert(numel(dir(fullfile(project_dir, 'figures', 'stage1', '*.png'))) == 3 ...
    && numel(dir(fullfile(project_dir, 'figures', 'stage2', '*.png'))) == 3 ...
    && numel(dir(fullfile(project_dir, 'figures', 'stage3', '*.png'))) == 6, ...
    'Unexpected number of stage plots.');
links = regexp(fileread(fullfile(project_dir, 'README.md')), ...
    '\[[^\]]*\]\((figures/[^)]+)\)', 'tokens');
for j = 1:numel(links)
    assert(isfile(fullfile(project_dir, links{j}{1})), 'README plot link is broken.');
end
