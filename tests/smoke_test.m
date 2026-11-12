% main also runs the model checks
addpath(fileparts(fileparts(mfilename('fullpath'))));
main;
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
