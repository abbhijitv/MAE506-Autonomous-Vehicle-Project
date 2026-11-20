function [u, memory] = pid_controller(cfg, v, memory)
% filtered derivative on speed avoids a reference derivative kick
e = cfg.v_ref - v;
alpha = exp(-cfg.dt/cfg.pid_filter_tau);
memory.derivative = alpha*memory.derivative ...
    + (1 - alpha)*(v - memory.last_speed)/cfg.dt;
feedforward = cfg.b*cfg.v_ref + cfg.c*cfg.v_ref*abs(cfg.v_ref) + cfg.d_eq;
raw = feedforward + cfg.pid_Kp*e + cfg.pid_Ki*memory.integral ...
    - cfg.pid_Kd*memory.derivative;
u = min(max(raw, cfg.F_min), cfg.F_max);

% integrate only when the error does not drive further into saturation
if ~((raw >= cfg.F_max && e > 0) || (raw <= cfg.F_min && e < 0))
    memory.integral = memory.integral + cfg.dt*e;
end
memory.last_speed = v;
end
