function model = vehicle_model(cfg)
% nonlinear states: x = [v; Fa], input u, output y = v
model.dynamics = @(x, u, d) ...
    [(x(2) - cfg.b*x(1) - cfg.c*x(1)*abs(x(1)) - d)/cfg.m;
     (-x(2) + u)/cfg.tau];

% constant-speed cruising equilibrium
model.x_eq = [cfg.v_eq;
    cfg.b*cfg.v_eq + cfg.c*cfg.v_eq*abs(cfg.v_eq) + cfg.d_eq];
model.u_eq = model.x_eq(2);
model.d_eq = cfg.d_eq;

% Jacobian model: delta_x = x - x_eq, delta_u = u - u_eq
% delta_x_dot = A*delta_x + B*delta_u + E*delta_d
% delta_y = C*delta_x, where delta_d = d - d_eq
drag_slope = cfg.b + 2*cfg.c*abs(cfg.v_eq);
model.A = [-drag_slope/cfg.m, 1/cfg.m;
           0,                  -1/cfg.tau];
model.B = [0; 1/cfg.tau];
model.C = [1, 0];
model.D = 0;
model.E = [-1/cfg.m; 0];

% create the continuous-time deviation models
model.ss = ss(model.A, model.B, model.C, model.D);
model.tf = tf(model.ss);
end
