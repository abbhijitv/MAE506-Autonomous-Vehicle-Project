% open-loop motion with constant force and steering
cfg = config();
x0 = [cfg.X0; cfg.Y0; cfg.psi0; cfg.v0; cfg.Fa0];
[t, x] = ode45(@(t, x) vehicle_2d_model(cfg, x, cfg.demo_force, ...
    cfg.demo_steering, cfg.demo_disturbance), cfg.t, x0);

% save plots without opening figure windows
figure_dir = fullfile(fileparts(mfilename('fullpath')), 'figures', 'stage1');
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

% the default force approaches the cruising speed gradually
trajectory_fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
trajectory_ax = axes('Parent', trajectory_fig);
plot(trajectory_ax, x(:, 1), x(:, 2), 'LineWidth', 1.5);
xlabel(trajectory_ax, 'X (m)');
ylabel(trajectory_ax, 'Y (m)');
title(trajectory_ax, 'Open-loop vehicle trajectory');
axis(trajectory_ax, 'equal');
grid(trajectory_ax, 'on');
drawnow nocallbacks;
exportgraphics(trajectory_ax, fullfile(figure_dir, 'vehicle_trajectory.png'), ...
    'Resolution', 300);
close(trajectory_fig);

speed_fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
speed_ax = axes('Parent', speed_fig);
plot(speed_ax, t, x(:, 4), 'LineWidth', 1.5);
xlabel(speed_ax, 'Time (s)');
ylabel(speed_ax, 'Speed (m/s)');
title(speed_ax, 'Open-loop vehicle speed');
grid(speed_ax, 'on');
drawnow nocallbacks;
exportgraphics(speed_ax, fullfile(figure_dir, 'vehicle_speed.png'), ...
    'Resolution', 300);
close(speed_fig);

heading_fig = figure('WindowStyle', 'normal', 'Visible', 'off', 'HandleVisibility', 'off');
heading_ax = axes('Parent', heading_fig);
plot(heading_ax, t, x(:, 3), 'LineWidth', 1.5);
xlabel(heading_ax, 'Time (s)');
ylabel(heading_ax, 'Heading (rad)');
title(heading_ax, 'Open-loop vehicle heading');
grid(heading_ax, 'on');
drawnow nocallbacks;
exportgraphics(heading_ax, fullfile(figure_dir, 'vehicle_heading.png'), ...
    'Resolution', 300);
close(heading_fig);
