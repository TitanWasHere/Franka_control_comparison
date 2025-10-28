function dx = robot_dynamics_bang_coast_bang(t, x, q0, qf, T_final, controller_func, controller_gains, true_params, uncertain_params, coast_fraction)
    % robot dynamics simulation for a bang-coast-bang trajectory
    % output dx=[dq; ddq]

    % default coast fraction
    if nargin < 10
        coast_fraction = 0.4;
    end

    % current state
    q = x(1:7);
    qd = x(8:14);

    % generate desired trajectory
    [q_des, qd_des, qdd_des] = generate_bang_coast_bang_trajectory(t, q0, qf, T_final, coast_fraction);

    % compute control torque using uncertain params
    tau = controller_func(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params);

    % get plant dynamics using true params
    [M, c, g] = recursive_newton_euler(q, qd, zeros(7,1), true_params);
    tau_f = compute_friction(qd, true_params);

    % check mass matrix conditioning
    %if cond(M) > 1e12
    %    warning('robot_dynamics_bang_coast_bang: Matrice massa M mal condizionata (cond=%.2e)', cond(M));
    %    M = M + 1e-6 * eye(7); % regularization
    %end

    % check for non-finite values in dynamics
    %if any(~isfinite(c)) || any(~isfinite(g)) || any(~isfinite(tau_f))
    %    warning('robot_dynamics_bang_coast_bang: Dinamiche non finite, usando valori zero');
    %    c(~isfinite(c)) = 0;
    %    g(~isfinite(g)) = 0;
    %    tau_f(~isfinite(tau_f)) = 0;
    %end
%
    %% check for non-finite control torque
    %if any(~isfinite(tau))
    %    warning('robot_dynamics_bang_coast_bang: Coppie controllo non finite, usando zero');
    %    tau(~isfinite(tau)) = 0;
    %end

    % robot dynamics solved for qdd
    qdd = M \ (tau - c - g - tau_f);

    % check for non-finite acceleration
    %if any(~isfinite(qdd))
    %    warning('robot_dynamics_bang_coast_bang: Accelerazioni non finite, usando zero');
    %    qdd(~isfinite(qdd)) = 0;
    %end

    % state derivative
    dx = [qd; qdd];

end