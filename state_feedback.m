function controller = state_feedback(cfg, model)
% integral state z_dot = reference speed - measured speed
controller.A_aug = [model.A, zeros(2, 1); -model.C, 0];
controller.B_aug = [model.B; 0];
controller.gain = place(controller.A_aug, controller.B_aug, cfg.feedback_poles);
controller.Kx = controller.gain(1:2);
controller.Ki = controller.gain(3);
controller.poles = eig(controller.A_aug - controller.B_aug*controller.gain);
% the two slow poles regulate speed; the faster pole accounts for actuator lag
controller.command = @(x, z) model.u_eq ...
    - controller.Kx*(x - model.x_eq) - controller.Ki*z;
assert(controller.Ki < 0, 'Integral gain must increase force for positive speed error.');
assert(max(abs(sort(controller.poles) - sort(cfg.feedback_poles(:)))) < 1e-8, ...
    'State-feedback pole placement did not match the requested poles.');
end
