function process_and_save_results(config, t_sim, x_sim, sim_time)
    fprintf('\n=== ANALYZING, PLOTTING, AND SAVING RESULTS ===\n');

    % unpack config
    controller_func = config.controller_func;
    trajectory_func = config.trajectory_func;
    trajectory_params = config.trajectory_params;
    resultsPath = config.resultsPath;
    plot_dir = config.plot_dir;
    traj_label = config.traj_label;
    T_final = config.T_final;
    controller_gains = config.controller_gains;
    uncertain_params = config.uncertain_params;

    %% performance analysis
    fprintf('Analyzing performance...\n');
    
    % extract sim data
    q_sim = x_sim(:, 1:7)';    % 7 x N_points
    qd_sim = x_sim(:, 8:14)';  % 7 x N_points
    n_points = length(t_sim);

    % desired trajectory for analysis
    q_desired = zeros(7, n_points);
    qd_desired = zeros(7, n_points);
    qdd_desired = zeros(7, n_points);
    for i = 1:n_points
        [q_desired(:,i), qd_desired(:,i), qdd_desired(:,i)] = trajectory_func(t_sim(i), trajectory_params{:});
    end

    % tracking errors
    e_pos = q_sim - q_desired;
    e_vel = qd_sim - qd_desired;

    % joint-space performance metrics
    max_pos_error_per_joint = max(abs(e_pos), [], 2);
    rms_pos_error_per_joint = sqrt(mean(e_pos.^2, 2));
    total_max_error = max(max_pos_error_per_joint);
    total_rms_error = sqrt(mean(rms_pos_error_per_joint.^2));
    fprintf('  Joint-space max error: %.4f rad (%.2f°)\n', total_max_error, rad2deg(total_max_error));
    fprintf('  Joint-space RMS error: %.4f rad (%.2f°)\n', total_rms_error, rad2deg(total_rms_error));

    %% trajectory-specific analysis
    switch config.trajectory_type
        case "bang_bang"
            fprintf('Performing bang-bang specific analysis...\n');
            switch_time_idx = find(t_sim >= T_final/2, 1);
            if ~isempty(switch_time_idx)
                q0 = trajectory_params{1};
                qf = trajectory_params{2};
                Delta_q = qf - q0;
                max_velocities = 2 * Delta_q / T_final;
                
                fprintf('  Characteristics at half time (t=%.2fs):\n', t_sim(switch_time_idx));
                fprintf('    Simulated Velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', qd_sim(:, switch_time_idx));
                fprintf('    Theoretical Max Vels: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', max_velocities);
            end

        case "bang_coast_bang"
            fprintf('Performing bang-coast-bang specific analysis...\n');
            COAST_FRACTION = trajectory_params{4};
            T_accel = T_final * (1 - COAST_FRACTION) / 2;
            T_coast = T_final * COAST_FRACTION;
            
            accel_end_idx = find(t_sim >= T_accel, 1);
            coast_end_idx = find(t_sim >= T_accel + T_coast, 1);
            
            if ~isempty(accel_end_idx) && ~isempty(coast_end_idx)
                q0 = trajectory_params{1};
                qf = trajectory_params{2};
                Delta_q = qf - q0;
                accelerations = Delta_q ./ (T_accel * (T_accel + T_coast));
                velocities_coast = accelerations * T_accel;

                fprintf('  End of acceleration (t=%.2fs):\n', t_sim(accel_end_idx));
                fprintf('    Simulated Velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', qd_sim(:, accel_end_idx));
                fprintf('    Theoretical Vels:     [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', velocities_coast);
                
                coast_velocities = qd_sim(:, accel_end_idx:coast_end_idx);
                coast_std = std(coast_velocities, 0, 2);
                fprintf('  During coast phase:\n');
                fprintf('    Std Dev of Velocity:  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad/s\n', coast_std);
            end
    end

    %% end-effector analysis
    fprintf('Calculating end-effector trajectories...\n');
    DH = get_DH();
    ee_pos_sim = zeros(3, n_points);
    ee_pos_des = zeros(3, n_points);

    for i = 1:n_points
        % Actual position
        T_sim = eye(4);
        for j = 1:7
            a = DH(j,1); alpha = DH(j,2); d = DH(j,3); theta = q_sim(j,i) + DH(j,4);
            T_i = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                   sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                   0           sin(alpha)             cos(alpha)            d;
                   0           0                      0                     1];
            T_sim = T_sim * T_i;
        end
        ee_pos_sim(:,i) = T_sim(1:3, 4);
        
        % Desired position
        T_des = eye(4);
        for j = 1:7
            a = DH(j,1); alpha = DH(j,2); d = DH(j,3); theta = q_desired(j,i) + DH(j,4);
            T_i = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                   sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                   0           sin(alpha)             cos(alpha)            d;
                   0           0                      0                     1];
            T_des = T_des * T_i;
        end
        ee_pos_des(:,i) = T_des(1:3, 4);
    end

    % end-effector performance metrics
    ee_error = sqrt(sum((ee_pos_sim - ee_pos_des).^2, 1)) * 1000; % [mm]
    max_ee_error = max(ee_error);
    rms_ee_error = sqrt(mean(ee_error.^2));
    final_ee_error = ee_error(end);

    fprintf('  End-effector max error: %.2f mm, RMS: %.2f mm, Final: %.2f mm\n', max_ee_error, rms_ee_error, final_ee_error);

    %% control torque computation
    fprintf('Calculating control torques...\n');
    tau_computed = zeros(length(t_sim), 7);
    for i = 1:length(t_sim)
        [q_d, qd_d, qdd_d] = trajectory_func(t_sim(i), trajectory_params{:});
        tau_computed(i,:) = controller_func(q_sim(:,i), qd_sim(:,i), q_d, qd_d, qdd_d, controller_gains, uncertain_params);
    end

    %% plot generation
    fprintf('Generating plots...\n');
    if ~exist(plot_dir, 'dir'), mkdir(plot_dir); end
    
    joint_names = {'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)', 'Forearm (J4)', ...
                   'Wrist1 (J5)', 'Wrist2 (J6)', 'Wrist3 (J7)'};
    
    % plot 1: joint position tracking (all 7 joints in one figure, stacked vertically)
    figure('Name', 'Joint Position Tracking', 'Position', [100 100 900 1200], 'Visible', 'off');
    for joint = 1:7
        subplot(7, 1, joint);
        plot(t_sim, q_desired(joint,:), 'r:', 'LineWidth', 1.5, 'DisplayName', 'q_{des}');
        hold on;
        plot(t_sim, q_sim(joint,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'q');
        ylabel('rad', 'FontSize', 9);
        if joint == 1
            title(sprintf('Joint Position Tracking - %s', joint_names{joint}), 'FontSize', 9);
            legend('show', 'Location', 'northeast', 'FontSize', 8, 'Orientation', 'horizontal');
        else
            title(joint_names{joint}, 'FontSize', 9);
        end
        if joint == 7
            xlabel('t', 'FontSize', 9);
        end
        grid on;
        xlim([t_sim(1) t_sim(end)]);
        
        % Set ylim to [-1, 1] if the data range is in the order of 10^-2 or smaller
        all_values = [q_desired(joint,:), q_sim(joint,:)];
        data_range = max(abs(all_values));
        if data_range <= 0.01
            ylim([-0.5, 0.5]);
        end
    end
    saveas(gcf, fullfile(plot_dir, sprintf('all_joints_tracking_%s.png', traj_label)));
    close(gcf);

    % plot 2: end-effector tracking
    figure('Name', 'End-Effector Tracking', 'Position', [100 100 1200 800], 'Visible', 'off');
    subplot(2, 2, 1);
    plot3(ee_pos_des(1,:), ee_pos_des(2,:), ee_pos_des(3,:), 'g--', 'LineWidth', 3, 'DisplayName', 'Desired');
    hold on;
    plot3(ee_pos_sim(1,:), ee_pos_sim(2,:), ee_pos_sim(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    plot3(ee_pos_sim(1,1), ee_pos_sim(2,1), ee_pos_sim(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(ee_pos_sim(1,end), ee_pos_sim(2,end), ee_pos_sim(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    title('End-Effector 3D Trajectory'); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    legend('show'); grid on; axis equal;
    % X, Y, Z components
    subplot(2, 2, 2); plot(t_sim, ee_pos_des(1,:), 'g--'); hold on; plot(t_sim, ee_pos_sim(1,:), 'b-'); title('X Position'); grid on;
    subplot(2, 2, 3); plot(t_sim, ee_pos_des(2,:), 'g--'); hold on; plot(t_sim, ee_pos_sim(2,:), 'b-'); title('Y Position'); grid on;
    subplot(2, 2, 4); plot(t_sim, ee_pos_des(3,:), 'g--'); hold on; plot(t_sim, ee_pos_sim(3,:), 'b-'); title('Z Position'); grid on;
    sgtitle(sprintf('End-Effector Tracking (Max Error: %.2f mm, RMS: %.2f mm)', max_ee_error, rms_ee_error));
    saveas(gcf, fullfile(plot_dir, sprintf('end_effector_tracking_%s.png', traj_label)));
    close(gcf);

    % plot 3: tracking errors
    figure('Name', 'Tracking Errors', 'Position', [100 100 1200 600], 'Visible', 'off');
    subplot(1, 2, 1);
    plot(t_sim, sqrt(sum(e_pos.^2, 1))*1000, 'r-');
    title(sprintf('Total Position Error\nMax: %.2f mrad, RMS: %.2f mrad', ...
                  max(sqrt(sum(e_pos.^2, 1)))*1000, sqrt(mean(sum(e_pos.^2, 1)))*1000));
    xlabel('Time [s]'); ylabel('Position Error ||e|| [mrad]'); grid on;
    subplot(1, 2, 2);
    plot(t_sim, sqrt(sum(e_vel.^2, 1))*1000, 'b-');
    title(sprintf('Total Velocity Error\nMax: %.2f mrad/s, RMS: %.2f mrad/s', ...
                  max(sqrt(sum(e_vel.^2, 1)))*1000, sqrt(mean(sum(e_vel.^2, 1)))*1000));
    xlabel('Time [s]'); ylabel('Velocity Error ||ė|| [mrad/s]'); grid on;
    saveas(gcf, fullfile(plot_dir, sprintf('position_velocity_errors_%s.png', traj_label)));
    close(gcf);

    % plot 4: control torques
    figure('Name', 'Control Torques', 'Position', [100 100 900 1200], 'Visible', 'off');
    for joint = 1:7
        subplot(7, 1, joint);
        plot(t_sim, tau_computed(:,joint), 'r-', 'LineWidth', 1.5);
        ylabel('Nm', 'FontSize', 9);
        if joint == 1
            title(sprintf('Control Torques - %s', joint_names{joint}), 'FontSize', 9);
        else
            title(joint_names{joint}, 'FontSize', 9);
        end
        if joint == 7
            xlabel('t', 'FontSize', 9);
        end
        grid on;
        xlim([t_sim(1) t_sim(end)]);
    end
    sgtitle('Control Torques for All Joints');
    saveas(gcf, fullfile(plot_dir, sprintf('control_torques_%s.png', traj_label)));
    close(gcf);

    fprintf('Plots saved.\n');

    %% save results
    results = struct();
    results.controller = func2str(controller_func);
    results.trajectory_type = config.trajectory_type;
    results.simulation_time = sim_time;

    % performance metrics
    results.performance.joint_max_error_rad = total_max_error;
    results.performance.joint_rms_error_rad = total_rms_error;
    results.performance.ee_max_error_mm = max_ee_error;
    results.performance.ee_rms_error_mm = rms_ee_error;
    results.performance.ee_final_error_mm = final_ee_error;

    % sim data
    results.simulation.t = t_sim;
    results.simulation.q = q_sim;
    results.simulation.qd = qd_sim;
    results.simulation.ee_pos = ee_pos_sim;
    results.simulation.tau = tau_computed';
    
    % desired trajectory data
    results.desired.q = q_desired;
    results.desired.qd = qd_desired;
    results.desired.qdd = qdd_desired;
    results.desired.ee_pos = ee_pos_des;

    % error data
    results.errors.pos = e_pos;
    results.errors.vel = e_vel;
    results.errors.ee_mm = ee_error;

    % configuration
    results.config = config;
    results.timestamp = datetime('now');

    [results_dir, ~, ~] = fileparts(resultsPath);
    if ~exist(results_dir, 'dir'), mkdir(results_dir); end
    save(resultsPath, 'results', '-v7.3');
    fprintf('\nResults saved to: %s\n', resultsPath);
end