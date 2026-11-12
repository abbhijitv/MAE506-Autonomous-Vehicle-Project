function cfg = config()
% preliminary modeling assumptions, not measured vehicle parameters

% vehicle parameters
cfg.m = 1500;                 % kg
cfg.b = 50;                   % N*s/m
cfg.c = 0.4;                  % N*s^2/m^2, illustrative quadratic drag
cfg.tau = 0.5;                % s, illustrative actuator time constant

% simulation settings
cfg.dt = 0.01;                % s
cfg.t_end = 120;              % s
cfg.t = 0:cfg.dt:cfg.t_end;
cfg.v0 = 0;                   % m/s
cfg.v_ref = 25;               % m/s
cfg.Fa0 = 0;                  % N, initial actuator force

% cruising equilibrium for linearization
cfg.v_eq = 25;                % m/s
cfg.d_eq = 0;                 % N

% opposing disturbance settings for later simulations
cfg.disturbance_start = 60;   % s
cfg.disturbance_force = 1500; % N

% actuator limits
cfg.F_max = 6000;             % N
cfg.F_min = -6000;            % N

% bicycle model geometry and initial conditions
cfg.L = 2.7;                  % m, wheelbase
cfg.steering_max = deg2rad(30); % rad
cfg.X0 = 0;                   % m
cfg.Y0 = 0;                   % m
cfg.psi0 = 0;                 % rad
cfg.delta0 = 0;               % rad

% reference path settings
cfg.path_length = 300;        % m
cfg.path_amplitude = 5;       % m
cfg.path_wavelength = 100;    % m

% constant inputs for the open-loop 2D demo
cfg.demo_force = 1500;        % N
cfg.demo_steering = cfg.delta0; % rad
cfg.demo_disturbance = 0;     % N

% small steps for linearized open-loop analysis
cfg.analysis_force_step = 100;       % N, commanded force increment at t = 0
cfg.analysis_disturbance_step = 100; % N, increment at disturbance_start

% PID cruise control, derivative on measured speed
cfg.pid_Kp = 900;             % N/(m/s)
cfg.pid_Ki = 120;             % N/m
cfg.pid_Kd = 250;             % N/(m/s^2)
cfg.pid_filter_tau = 0.1;     % s

% integral state feedback and observer pole locations
cfg.feedback_poles = [-0.25, -0.4, -1.5]; % 1/s
cfg.observer_poles = [-2.5, -3.5];         % 1/s
cfg.observer_duration = 20;   % s, local test near cruise equilibrium
cfg.observer_speed_offset = -1; % m/s, actual speed relative to equilibrium
cfg.observer_initial_error = [-2; -500]; % m/s and N, estimate minus actual

% pure pursuit at a constant longitudinal equilibrium speed
cfg.path_speed = 10;          % m/s
cfg.path_lookahead = 8;       % m
cfg.path_lateral_offset = 1;  % m
cfg.path_max_time = 1.5*cfg.path_length/cfg.path_speed; % s
end
