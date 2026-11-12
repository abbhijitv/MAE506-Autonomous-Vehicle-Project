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
end
