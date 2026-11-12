function dx = vehicle_2d_model(cfg, x, u, delta, d)
% states: [X; Y; psi; v; Fa], with X and Y at the rear axle
validateattributes(x, {'numeric'}, {'real', 'finite', 'size', [5, 1]});
validateattributes(cfg.m, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(cfg.tau, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(cfg.L, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(cfg.b, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(cfg.c, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(u, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(delta, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(d, {'numeric'}, {'real', 'finite', 'scalar'});
assert(abs(delta) < pi/2, 'Steering angle must lie between -pi/2 and pi/2.');

psi = x(3);
v = x(4);
Fa = x(5);

% kinematics and unchanged longitudinal dynamics
dx = [v*cos(psi);
      v*sin(psi);
      (v/cfg.L)*tan(delta);
      (Fa - cfg.b*v - cfg.c*v*abs(v) - d)/cfg.m;
      (-Fa + u)/cfg.tau];
end
