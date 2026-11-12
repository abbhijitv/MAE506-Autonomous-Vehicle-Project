function [delta, lateral_error] = path_tracking_controller(cfg, x, path)
% pure pursuit aims at a point ahead of the nearest path sample
[~, nearest] = min((path.X - x(1)).^2 + (path.Y - x(2)).^2);
target_s = min(path.s(nearest) + cfg.path_lookahead, path.s(end));
target_X = interp1(path.s, path.X, target_s);
target_Y = interp1(path.s, path.Y, target_s);
dx = target_X - x(1);
dy = target_Y - x(2);
distance = hypot(dx, dy);
alpha = atan2(dy, dx) - x(3);
delta = atan2(2*cfg.L*sin(alpha), max(distance, eps));
delta = min(max(delta, -cfg.steering_max), cfg.steering_max);

% signed error along the nearest reference sample's normal
lateral_error = -(x(1) - path.X(nearest))*sin(path.psi(nearest)) ...
    + (x(2) - path.Y(nearest))*cos(path.psi(nearest));
end
