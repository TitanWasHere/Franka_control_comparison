function dx = robot_dynamics(t, x, trajectory_func, trajectory_params, controller_func, controller_gains, true_params, uncertain_params)
    % current state
    q = x(1:7);
    qd = x(8:14);

    % generate desired trajectory
    [q_des, qd_des, qdd_des] = trajectory_func(t, trajectory_params{:});

    % compute control torque using uncertain params
    tau = controller_func(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params);

    % get plant dynamics using true parameters
    [M, c, g] = recursive_newton_euler(q, qd, zeros(7,1), true_params);
    tau_f = compute_friction(qd, true_params);

    % robot dynamics solved for qdd
    % add a small regularization term for numerical stability if M is ill-conditioned
    if rcond(M) < 1e-12
        M = M + 1e-6 * eye(7);
    end
    qdd = M \ (tau - c - g - tau_f);

    % final check for non-finite accelerations
    %if any(~isfinite(qdd))
    %    warning('robot_dynamics: Non-finite accelerations computed. Setting to zero.');
    %    qdd(isinf(qdd) | isnan(qdd)) = 0;
    %end

    % state derivative
    dx = [qd; qdd];
end