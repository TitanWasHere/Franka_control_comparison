function [config, controller_gains, uncertain_params, true_params_vec, x0] = setup_simulation(config)
    fprintf('=== CONFIGURING SIMULATION ===\n');

    %% path setup
    try
        this_script_path = mfilename('fullpath');
        [script_dir, ~, ~] = fileparts(this_script_path);
        [project_root, ~, ~] = fileparts(script_dir);
    catch
        project_root = pwd; 
        warning('Could not determine project root automatically. Assuming current directory is project root.');
    end
    
    addpath(fullfile(project_root, 'lib'));
    run(fullfile(project_root, 'lib', 'setup_numerical_parameters.m'));
    uncertain_params = setup_uncertainty_levels(true_params_vec).inertia_coriolis;

    %% construct file paths for results and plots
    sub_path = sprintf("%s/", config.controller_type);
    if config.matched_start, sub_path = strcat(sub_path, "matched/"); else, sub_path = strcat(sub_path, "mismatched/"); end
    if config.optimized_gains, sub_path = strcat(sub_path, "optimized/"); else, sub_path = strcat(sub_path, "unoptimized/"); end
    sub_path = strcat(sub_path, sprintf("%s/", config.trajectory_type));

    switch config.trajectory_type
        case "quintic", traj_suffix = "quintic";
        case "bang_bang", traj_suffix = "bb";
        case "bang_coast_bang", traj_suffix = "bcb";
    end
    file_name = sprintf("%s_%s_%s.mat", config.controller_type, traj_suffix, config.traj_label);
    
    config.resultsPath = fullfile(project_root, 'results', sub_path, file_name);
    config.plot_dir = fullfile(project_root, 'plots', sub_path);
    fprintf('  Controller: %s, Trajectory: %s (%s)\n', config.controller_type, config.trajectory_type, config.traj_label);
    fprintf('  Results will be saved to: %s\n', config.resultsPath);

    %% controller gains
    if config.controller_type == "FBL"
        controller_gains = FBL_gains(config.optimized_gains);
    elseif config.controller_type == "PBC"
        controller_gains = PBC_gains(config.optimized_gains);
    else
        error('Unsupported controller type: %s. Cannot select gains.', config.controller_type);
    end
    
    config.controller_gains = controller_gains;
    config.uncertain_params = uncertain_params;

    %% trajectory parameters
    if config.traj_label == 'A'
        q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]'; 
        qf = q0 + [0.3, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3]';
    else % 'B'
        q0 = [-pi/2, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]';
        qf = q0 + [0.5, -0.5, pi/2, -pi/2, 0.0,pi, 0.5]';
    end

    % safety limits
    limits = setup_robot_limits();
    q_max_safe = limits.q_max - limits.safety_margin;
    q_min_safe = limits.q_min + limits.safety_margin;
    qf = max(q_min_safe, min(qf, q_max_safe));
    
    COAST_FRACTION = 0.4;
    switch config.trajectory_type
        case "quintic"
            config.trajectory_func = @generate_quintic_trajectory;
            config.trajectory_params = {q0, qf, config.T_final};
        case "bang_bang"
            config.trajectory_func = @generate_bang_bang_trajectory;
            config.trajectory_params = {q0, qf, config.T_final};
        case "bang_coast_bang"
            config.trajectory_func = @generate_bang_coast_bang_trajectory;
            config.trajectory_params = {q0, qf, config.T_final, COAST_FRACTION};
    end

    %% initial state
    if config.matched_start, q0_actual = q0; else, q0_actual = q0 + deg2rad([5,-10,0,8,0,0,0]'); end
    x0 = [q0_actual; zeros(7,1)];

    fprintf('Setup complete.\n\n');
end