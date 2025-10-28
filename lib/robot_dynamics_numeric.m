function dx = robot_dynamics_numeric(t, x, q0, qf, T_final, controller_func, controller_gains, true_params, uncertain_params)
    % robot dynamics for a quintic trajectory
    % output dx=[dq; ddq]

    % current state
    q = x(1:7);
    qd = x(8:14);

    % generate desired trajectory
    [q_des, qd_des, qdd_des] = generate_trajectory(t, q0, qf, T_final);

    % compute control torque using uncertain params
    tau = controller_func(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params);

    % get plant dynamics using true params
    [M, c, g] = recursive_newton_euler(q, qd, zeros(7,1), true_params);
    tau_f = compute_friction(qd, true_params);

    % check mass matrix conditioning
    %if cond(M) > 1e12
    %    warning('robot_dynamics_bang_coast_bang: Matrice massa M mal condizionata (cond=%.2e)', cond(M));
    %   M = M + 1e-6 * eye(7); % regularization
    %end

    % check for non-finite values in dynamics
    %if any(~isfinite(c)) || any(~isfinite(g)) || any(~isfinite(tau_f))
    %    warning('robot_dynamics_numeric: dinamiche non finite, usando zero');
    %    c = zeros(7,1);
    %    g = zeros(7,1); 
    %    tau_f = zeros(7,1);
    %end

    % robot dynamics solved for qdd
    qdd = M \ (tau - c - g - tau_f);

    % state derivative
    dx = [qd; qdd];

end