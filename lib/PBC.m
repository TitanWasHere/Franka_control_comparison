function tau = PBC(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params)
    % controller gains
    Kd = controller_gains.Kd;
    Lambda = controller_gains.Lambda;

    % tracking errors
    e_pos = q_des - q;
    e_vel = qd_des - qd;
    s = e_vel + Lambda*e_pos;

    % reference velocity/acceleration
    qdr = qd_des + Lambda*e_pos;
    qddr = qdd_des + Lambda*e_vel;

    % get M(q), g(q), and c(q, qd) for the actual velocity
    [M, c_qd, g] = recursive_newton_euler(q, qd, zeros(7,1), uncertain_params);
    tau_f = compute_friction(qd, uncertain_params);

    % compute mixed Coriolis term C(q, qd)*qdr
    [~, c_qdr, ~] = recursive_newton_euler(q, qdr, zeros(7,1), uncertain_params);
    [~, c_sum, ~] = recursive_newton_euler(q, qd + qdr, zeros(7,1), uncertain_params);
    C_qdr = 0.5*(c_sum - c_qd - c_qdr);

    % control torque
    tau = M*qddr + C_qdr + g + tau_f + Kd*s;

    % torque saturation
    tau_max = [87, 87, 87, 87, 12, 12, 12]';
    tau = max(-tau_max, min(tau_max, tau));
end