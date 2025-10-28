function tau = FBL(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params)
    % controller gains
    Kp = controller_gains.Kp;
    Kd = controller_gains.Kd;

    % compute dynamics
    [M, c, g] = recursive_newton_euler(q, qd, zeros(7,1), uncertain_params);
    tau_f = compute_friction(qd, uncertain_params);
    
    % tracking errors
    e_pos = q_des - q;
    e_vel = qd_des - qd;
    
    % auxiliary input (feedforward + PD)
    a = qdd_des + Kp * e_pos + Kd * e_vel;
    
    % control torque
    tau = M * a + c + g + tau_f;
    
    % torque saturation
    tau_max = [87, 87, 87, 87, 12, 12, 12]';
    tau = max(-tau_max, min(tau_max, tau));
end