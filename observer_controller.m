function observer = observer_controller(cfg, model)
% estimates are actual states; the internal model uses deviations
observer.L = place(model.A', model.C', cfg.observer_poles)';
observer.poles = eig(model.A - observer.L*model.C);
observer.dynamics = @(xhat, v, u) model.A*(xhat - model.x_eq) ...
    + model.B*(u - model.u_eq) + observer.L*(v - model.C*xhat);
assert(max(abs(sort(observer.poles) - sort(cfg.observer_poles(:)))) < 1e-8, ...
    'Observer pole placement did not match the requested poles.');
end
