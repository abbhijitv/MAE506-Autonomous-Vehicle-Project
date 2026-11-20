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

% actuator limits
cfg.F_max = 6000;             % N
cfg.F_min = -6000;            % N

% PID cruise control, derivative on measured speed
cfg.pid_Kp = 900;             % N/(m/s)
cfg.pid_Ki = 120;             % N/m
cfg.pid_Kd = 250;             % N/(m/s^2)
cfg.pid_filter_tau = 0.1;     % s

% Performance specifications
cfg.feedback_settling_time = 2; % s
cfg.feedback_overshoot = 7.5;   % percent

% comparison metric settings
cfg.rise_limits = [0.1, 0.9];
cfg.settling_threshold = 0.02;
cfg.steady_state_window = 5;    % s

end
