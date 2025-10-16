function tau = PBC_numeric(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params)
    % controller gains
    Kp = controller_gains.Kp;
    Kd = controller_gains.Kd;
    Lambda = controller_gains.Lambda;

    % tracking errors
    e_pos = q_des - q;
    e_vel = qd_des - qd;

    % reference velocity/acceleration and error signal
    qdr = qd_des + Lambda*e_pos;
    qddr = qdd_des + Lambda*e_vel;
    s = e_vel + Lambda*e_pos;

    % get M(q), g(q)
    [M, ~, ~, tau_f] = fast_dynamics(q, qd, uncertain_params);
    g = get_gravity_vector(q, uncertain_params);

    % compute mixed Coriolis term C(q, qd)*qdr
    c_qd = get_coriolis_vector(q, qd, uncertain_params);
    c_qdr = get_coriolis_vector(q, qdr, uncertain_params);
    c_sum = get_coriolis_vector(q, qd + qdr, uncertain_params);
    C_qdr = 0.5*(c_sum - c_qd - c_qdr);

    % control law
    %tau = M*qddr + C_qdr + g + tau_f + Kp*e_pos + Kd*e_vel;
    tau = M*qddr + C_qdr + g + tau_f + Kd*s;

    % safety check
    if any(~isfinite(tau))
        warning('PBC_numeric: tau is not finite. Setting to zero.');
        tau = zeros(7,1);
    end

    % torque saturation
    tau_max = [87, 87, 87, 87, 12, 12, 12]';
    tau = max(-tau_max, min(tau_max, tau));
end

function c = get_coriolis_vector(q, qd, params,g)
    [DH, masses, r_com, I_com] = get_robot_parameters(params);
    c_plus_g = newton_euler_forward(q, qd, zeros(7,1), DH, masses, r_com, I_com, [0;0;0]);
    g = get_gravity_vector(q, params);
    c = c_plus_g - g;
end

function g = get_gravity_vector(q, params)
    [DH, masses, r_com, I_com] = get_robot_parameters(params);
    g = newton_euler_forward(q, zeros(7,1), zeros(7,1), DH, masses, r_com, I_com, [0;0;0]);
end