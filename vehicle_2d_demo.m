% open-loop motion with constant force and steering
cfg = config();
x0 = [cfg.X0; cfg.Y0; cfg.psi0; cfg.v0; cfg.Fa0];
[t, x] = ode45(@(t, x) vehicle_2d_model(cfg, x, cfg.demo_force, ...
    cfg.demo_steering, cfg.demo_disturbance), cfg.t, x0);

% the default force approaches the cruising speed gradually
trajectory_fig = figure;
plot(x(:, 1), x(:, 2), 'LineWidth', 1.5);
xlabel('X (m)');
ylabel('Y (m)');
title('Open-loop vehicle trajectory');
axis equal;
grid on;

speed_fig = figure;
plot(t, x(:, 4), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Open-loop vehicle speed');
grid on;

heading_fig = figure;
plot(t, x(:, 3), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Heading (rad)');
title('Open-loop vehicle heading');
grid on;

% save the simulation plots
if ~exist('figures', 'dir')
    mkdir('figures');
end

exportgraphics(trajectory_fig, 'figures/vehicle_trajectory.png', ...
    'Resolution', 300);
exportgraphics(speed_fig, 'figures/vehicle_speed.png', ...
    'Resolution', 300);
exportgraphics(heading_fig, 'figures/vehicle_heading.png', ...
    'Resolution', 300);
