%% PASSIVITY-BASED CONTROL - TRAJECTORY PLOT FOR ALL JOINTS

clear; clc; close all;

fprintf('=== PBC Trajectory Tracking Analysis for All Joints ===\n\n');

%% 1. SETUP ENVIRONMENT & PARAMETERS
% Add library path and load numerical parameters for the robot
script_folder = fileparts(mfilename('fullpath')); 
lib_folder = fullfile(script_folder, '..', 'lib');
addpath(lib_folder);

try
    run(fullfile(lib_folder, 'setup_numerical_parameters.m'));
    fprintf('✓ Robot parameters loaded.\n');
catch ME
    fprintf('ERROR: Could not run the setup script. Ensure it is in the lib folder.\n');
    rethrow(ME);
end

% Setup controller gains, including the Lambda matrix for PBC
controller_gains = setup_controller_gains();
fprintf('✓ Controller gains initialized.\n');
fprintf('  Kp = diag([%s])\n', sprintf('%.0f, ', diag(controller_gains.Kp)));
fprintf('  Kd = diag([%s])\n', sprintf('%.0f, ', diag(controller_gains.Kd)));
fprintf('  Lambda = diag([%s])\n', sprintf('%.0f, ', diag(controller_gains.Lambda)));


%% 2. TRAJECTORY DEFINITION
T_final = 8.0;      
%q0 = [3*pi/2, -pi/4, 0, -3*pi/4, pi/4, pi/2, pi/4]'; 
q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]';
qf = q0 + [0.5, -0.5, pi/2, -pi/2, 0.0, pi, 0.5]';

% Ensure the final configuration respects the robot's physical joint limits
limits = setup_robot_limits();
q_max_safe = limits.q_max - limits.safety_margin;
q_min_safe = limits.q_min + limits.safety_margin;
qf = max(q_min_safe, min(qf, q_max_safe));
fprintf('Trajectory defined from q0 to qf over %.1f seconds.\n', T_final);

%% 3. SETUP SIMULATION
q0_actual = q0 + deg2rad([5; -10; 0; 8; 0; 0; 0]);
x0 = [q0_actual; zeros(7,1)]; 
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'MaxStep', 0.01);

% Use parameters with 20% uncertainty for the controller
uncertainty_struct = setup_uncertainty_levels(true_params_vec);
uncertain_params = uncertainty_struct.full; 
fprintf('Using 20%% parameter uncertainty for the controller.\n');

%% 4. RUN SIMULATION
fprintf('\nRunning PBC simulation...\n');
tic;
[t_sim, x_sim] = ode15s(@(t, x) robot_dynamics_numeric(t, x, q0, qf, T_final, @PBC_numeric, ...
                                                      controller_gains, true_params_vec, uncertain_params), ...
                       [0 T_final], x0, options);
sim_time = toc;
fprintf('✓ Simulation completed in %.3f seconds.\n', sim_time);

%% 5. DATA POST-PROCESSING
fprintf('Post-processing data for plotting...\n');
q_sim = x_sim(:, 1:7)';    
qd_sim = x_sim(:, 8:14)';
n_points = length(t_sim);
q_desired = zeros(7, n_points);
qd_desired = zeros(7, n_points);

for i = 1:n_points
    [q_desired(:,i), qd_desired(:,i), ~] = generate_trajectory(t_sim(i), q0, qf, T_final);
end

%% 6. PLOTTING RESULTS
fprintf('Generating plot...\n');

figure('Name', 'PBC Trajectory Tracking for All Joints', 'Position', [150 150 1100 700], 'Color', 'w');
hold on;

% Define a color order for the 7 joints for better visualization
colors = lines(7);

% Loop through each joint to plot desired and actual trajectories
legend_handles = gobjects(7,1); % Use this for a clean legend
for i = 1:7
    % Plot desired trajectory (thicker, dashed line)
    plot(t_sim, rad2deg(q_desired(i,:)), '--', 'Color', colors(i,:), 'LineWidth', 2.5, 'HandleVisibility', 'off');
    
    % Plot actual trajectory (thinner, solid line)
    legend_handles(i) = plot(t_sim, rad2deg(q_sim(i,:)), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Joint %d', i));
end

hold off;

% --- Customize Plot Appearance ---
title('PBC: Desired vs. Actual Joint Trajectories (with 20% uncertainty)', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Joint Position [deg]', 'FontSize', 12, 'FontWeight', 'bold');
legend(legend_handles, 'Location', 'eastoutside', 'FontSize', 10);
grid on;
box on;
set(gca, 'FontSize', 12, 'Layer', 'top');

fprintf('✓ Plot generated successfully.\n');

%% 7. SAVE RESULTS FOR ANIMATION
% --- MODIFIED SECTION ---
fprintf('\nSaving results for animation...\n');
results = struct();
results.controller = 'PBC_numeric';
results.simulation_time = sim_time;
results.simulation = struct('t', t_sim, 'q', q_sim, 'qd', qd_sim);
results.desired = struct('q', q_desired, 'qd', qd_desired);
results.timestamp = datetime('now');

% Construct the full path to save the file in the same directory as the script
base_filename = 'PBC_numeric_results.mat';
full_save_path = fullfile(script_folder, base_filename);

save(full_save_path, 'results', '-v7.3');

fprintf('✓ Results saved to: %s\n', full_save_path);
fprintf('🎉 Script finished! You can now run animate_PBC.m.\n');