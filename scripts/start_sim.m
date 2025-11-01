function start_sim(type, traj, traj_label, matched, optimized, T_final, uncertainty)
    config.controller_type = type;%"PBC";
    config.trajectory_type = traj;%"quintic";
    config.traj_label      = traj_label;%'A';
    config.matched_start   = matched;%true;
    config.optimized_gains = optimized;%false;
    config.T_final         = T_final;%8.0;
    config.uncertainty_level = uncertainty; %"high";

    [config, controller_gains, uncertain_params, true_params_vec, x0] = setup_simulation(config);
    if config.controller_type == "FBL"
        config.controller_func = @FBL;
    elseif config.controller_type == "PBC"
        config.controller_func = @PBC;
    end

    fprintf('=== RUNNING SIMULATION ===\n');
    tic;
    options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'MaxStep', 0.01);
    [t_sim, x_sim] = ode15s(@(t, x) robot_dynamics(t, x, config.trajectory_func, config.trajectory_params, ...
                                                config.controller_func, controller_gains, true_params_vec, uncertain_params), ...
                        [0 config.T_final], x0, options);
    sim_time = toc;
    fprintf('Simulation completed in %.3f seconds.\n', sim_time);
    process_and_save_results(config, t_sim, x_sim, sim_time);
end