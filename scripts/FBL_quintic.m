%% SIMPLE TRAJECTORY USING FEEDBACK LINEARIZATION - PURE NUMERIC VERSION

clear; clc; close all;

fprintf('=== QUINTIC TRAJECTORY ===\n\n');
fprintf('[Algorithm] Newton-Euler\n');

%% [! IMPORTANT] DEFINING CONSTANTS
TRAJ_A=true;
METCHED_START = true;
OPTIMIZED_GAINS = true;
path = "";

if MATCHED_START
    path = strcat(path, "matched/");
else
    path = strcat(path, "mismatched/");
end

if OPTIMIZED_GAINS
    path = strcat(path, "optimized/");
else
    path = strcat(path, "unoptimized/");
end

path = strcat(path, "quintic/");

resultsPath = "../results/" + path;
plot_dir = "../plots/" + path;

if TRAJ_A
    resultsPath = strcat(resultsPath, "FBL_quintic_A.mat");
    traj = "A";
else
    resultsPath = strcat(resultsPath, "FBL_quintic_B.mat");
    traj = "B";
end



%% Load numerical parameters
addpath("../lib");
run("../lib/setup_numerical_parameters.m");

controller_gains = setup_controller_gains();

if OPTIMIZED_GAINS
    controller_gains.Kp = controller_gains.Kp * 2;
    controller_gains.Kd = controller_gains.Kd + 10;
else
    controller_gains.Kp = controller_gains.Kp;
    controller_gains.Kd = controller_gains.Kd;
end

fprintf('Gains FBL numerici:\n');
fprintf('  Kp = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kp));
fprintf('  Kd = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kd));

%% TRAJECTORY DEFINITION

if TRAJ_A
    q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]'; 
    qf = q0 + [0.3, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3]';
else
    q0 = [3*pi/2, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]';
    qf = q0 + [0.5, -0.5, pi/2, -pi/2, 0.0,pi, 0.5]';
end

limits = setup_robot_limits();
q_max_safe = limits.q_max - limits.safety_margin;
q_min_safe = limits.q_min + limits.safety_margin;
qf = max(q_min_safe, min(qf, q_max_safe));


%% SETUP SIMULATION
T_final = 8.0;

if METCHED_START
    q0_actual = q0;
else
    q0_actual = q0 + deg2rad([5,-10,0,8,0,0,0]');
end
x0 = [q0_actual; zeros(7,1)];

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'MaxStep', 0.01, ...
                 'Stats', 'off', 'InitialStep', 1e-4, 'Refine', 1);

uncertain_params = setup_uncertainty_levels(true_params_vec);
uncertain_params = uncertain_params.inertia_coriolis; 

%% SIMULAZIONE CON ALGORITMI NUMERICI PURI

fprintf('\n=== FBL CALCULATING ===\n');

tic;
[t_sim, x_sim] = ode45(@(t, x) robot_dynamics_numeric(t, x, q0, qf, T_final, @FBL_stable, ...
                                                      controller_gains, true_params_vec, uncertain_params), ...
                       [0 T_final], x0, options);
sim_time = toc;

fprintf('✓ Simulation completed in %.3f seconds\n', sim_time);
fprintf('  Simulated points: %d\n', length(t_sim));
fprintf('  Mean frequency: %.1f Hz\n', length(t_sim)/T_final);

%% ESTRAZIONE DATI E ANALISI
fprintf('\n=== Results analysis ===\n');

q_sim = x_sim(:, 1:7)';    % 7 x N_points
qd_sim = x_sim(:, 8:14)';  % 7 x N_points

% Genera traiettoria desiderata per analisi
n_points = length(t_sim);
q_desired = zeros(7, n_points);
qd_desired = zeros(7, n_points);
qdd_desired = zeros(7, n_points);

for i = 1:n_points
    [q_desired(:,i), qd_desired(:,i), qdd_desired(:,i)] = generate_trajectory(t_sim(i), q0, qf, T_final);
end

% Calcola errori (effettivo - desiderato)
e_pos = q_sim - q_desired;
e_vel = qd_sim - qd_desired;

% Metriche di performance
max_pos_error = max(abs(e_pos), [], 2);
rms_pos_error = sqrt(mean(e_pos.^2, 2));
total_max_error = max(max_pos_error);
total_rms_error = sqrt(mean(rms_pos_error.^2));

fprintf('Performance FBL numerico:\n');
fprintf('  Errore massimo assoluto: %.4f rad (%.2f°)\n', total_max_error, rad2deg(total_max_error));
fprintf('  Errore RMS globale: %.4f rad (%.2f°)\n', total_rms_error, rad2deg(total_rms_error));

%% SETUP DIRECTORY PLOTS

if ~exist(plot_dir, 'dir')
    mkdir(plot_dir);
end

%% CALCOLO END-EFFECTOR POSITIONS
fprintf('\n=== End-effector position calculation ===\n');

DH = get_DH();

ee_pos_sim = zeros(3, n_points);
ee_pos_des = zeros(3, n_points);

for i = 1:n_points
    % Posizione effettiva
    T_sim = eye(4);
    for j = 1:7
        a = DH(j, 1); alpha = DH(j, 2); d = DH(j, 3); theta = q_sim(j,i) + DH(j, 4);
        T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1];
        T_sim = T_sim * T_i;
    end
    ee_pos_sim(:,i) = T_sim(1:3, 4);
    
    % Posizione desiderata
    T_des = eye(4);
    for j = 1:7
        a = DH(j, 1); alpha = DH(j, 2); d = DH(j, 3); theta = q_desired(j,i) + DH(j, 4);
        T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1];
        T_des = T_des * T_i;
    end
    ee_pos_des(:,i) = T_des(1:3, 4);
end

% Calcola errore end-effector
ee_error = sqrt(sum((ee_pos_sim - ee_pos_des).^2, 1)) * 1000; % in mm
max_ee_error = max(ee_error);
rms_ee_error = sqrt(mean(ee_error.^2));

fprintf('✓ Error end-effector - Max: %.2f mm, RMS: %.2f mm\n', max_ee_error, rms_ee_error);

%% CALCOLO COPPIE DI CONTROLLO
fprintf('Calcolo coppie di controllo...\n');
tau_computed = zeros(length(t_sim), 7);
for i = 1:length(t_sim)
    [q_d, qd_d, qdd_d] = generate_trajectory(t_sim(i), q0, qf, T_final);
    tau_computed(i,:) = FBL_numeric(q_sim(:,i), qd_sim(:,i), q_d, qd_d, qdd_d, controller_gains, uncertain_params);
end

%% GENERAZIONE GRAFICI E SALVATAGGIO
fprintf('\n=== GENERAZIONE E SALVATAGGIO GRAFICI ===\n');

% 1. TRACKING JOINT POSITIONS (7 separate plots)
joint_names = {'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)', 'Forearm (J4)', ...
               'Wrist1 (J5)', 'Wrist2 (J6)', 'Wrist3 (J7)'};

for joint = 1:7
    figure('Name', sprintf('Joint %d Tracking', joint), 'Position', [100 100 800 600]);
    plot(t_sim, rad2deg(q_desired(joint,:)), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired');
    hold on;
    plot(t_sim, rad2deg(q_sim(joint,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    title(sprintf('Joint %d Position Tracking - %s', joint, joint_names{joint}), 'FontSize', 14);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Angle [deg]', 'FontSize', 12);
    legend('show', 'FontSize', 12, 'Location', 'best');
    grid on; grid minor;
    
    % Add error info in textbox
    joint_error_max = max_pos_error(joint);
    joint_error_rms = rms_pos_error(joint);
    annotation('textbox', [0.15, 0.8, 0.3, 0.1], 'String', ...
               sprintf('Max Error: %.2f°\nRMS Error: %.2f°', ...
                      rad2deg(joint_error_max), rad2deg(joint_error_rms)), ...
               'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');

    saveas(gcf, fullfile(plot_dir, sprintf('joint_%d_tracking_%s.png', joint, traj)));
    close(gcf);
end

% 2. END-EFFECTOR TRACKING
figure('Name', 'End-Effector Tracking', 'Position', [100 100 1200 800]);

% 3D trajectory
subplot(2, 2, 1);
plot3(ee_pos_des(1,:), ee_pos_des(2,:), ee_pos_des(3,:), 'g--', 'LineWidth', 3, 'DisplayName', 'Desired');
hold on;
plot3(ee_pos_sim(1,:), ee_pos_sim(2,:), ee_pos_sim(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
plot3(ee_pos_sim(1,1), ee_pos_sim(2,1), ee_pos_sim(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(ee_pos_sim(1,end), ee_pos_sim(2,end), ee_pos_sim(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
title('End-Effector 3D Trajectory', 'FontSize', 14);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('show', 'Location', 'best'); grid on; axis equal;

% X, Y, Z components
subplot(2, 2, 2);
plot(t_sim, ee_pos_des(1,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired X');
hold on;
plot(t_sim, ee_pos_sim(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual X');
title('End-Effector X Position');
xlabel('Time [s]'); ylabel('X [m]'); legend('show'); grid on;

subplot(2, 2, 3);
plot(t_sim, ee_pos_des(2,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired Y');
hold on;
plot(t_sim, ee_pos_sim(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Y');
title('End-Effector Y Position');
xlabel('Time [s]'); ylabel('Y [m]'); legend('show'); grid on;

subplot(2, 2, 4);
plot(t_sim, ee_pos_des(3,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired Z');
hold on;
plot(t_sim, ee_pos_sim(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Z');
title('End-Effector Z Position');
xlabel('Time [s]'); ylabel('Z [m]'); legend('show'); grid on;

sgtitle(sprintf('End-Effector Tracking (Max Error: %.2f mm, RMS: %.2f mm)', max_ee_error, rms_ee_error), 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, sprintf('end_effector_tracking_%s.png', traj)));
close(gcf);

% 3. VELOCITY TRACKING
figure('Name', 'Velocity Tracking', 'Position', [100 100 1400 1000]);

for joint = 1:7
    subplot(3, 3, joint);
    plot(t_sim, rad2deg(qd_desired(joint,:)), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired');
    hold on;
    plot(t_sim, rad2deg(qd_sim(joint,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    
    title(sprintf('Joint %d Velocity - %s', joint, joint_names{joint}), 'FontSize', 12);
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Velocity [deg/s]', 'FontSize', 10);
    legend('show', 'Location', 'best');
    grid on; grid minor;
    
    % Calculate velocity error statistics for this joint
    vel_error = qd_desired(joint,:) - qd_sim(joint,:);
    max_vel_error = max(abs(vel_error));
    rms_vel_error = sqrt(mean(vel_error.^2));
    
    % Add error statistics as text
    text(0.02, 0.98, sprintf('Max Err: %.2f°/s\nRMS Err: %.2f°/s', ...
         rad2deg(max_vel_error), rad2deg(rms_vel_error)), ...
         'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'BackgroundColor', 'white', 'FontSize', 9);
end

% Add overall velocity tracking statistics
total_vel_error_norm = sqrt(sum((qd_desired - qd_sim).^2, 1));
max_total_vel_error = max(total_vel_error_norm);
rms_total_vel_error = sqrt(mean(total_vel_error_norm.^2));

sgtitle(sprintf('Joint Velocity Tracking (Max Total Error: %.2f°/s, RMS Total Error: %.2f°/s)', ...
        rad2deg(max_total_vel_error), rad2deg(rms_total_vel_error)), 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, sprintf('velocity_tracking_%s.png', traj)));
close(gcf);

% 4. POSITION AND VELOCITY ERRORS
figure('Name', 'Position and Velocity Errors', 'Position', [100 100 1200 600]);

% Total position error
subplot(1, 2, 1);
error_norm = sqrt(sum(e_pos.^2, 1));
plot(t_sim, error_norm*1000, 'r-', 'LineWidth', 2.5);
title(sprintf('Total Position Error\nMax: %.2f mrad, RMS: %.2f mrad', ...
              max(error_norm)*1000, sqrt(mean(error_norm.^2))*1000), 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Position Error ||e|| [mrad]', 'FontSize', 12);
grid on; grid minor;

% Total velocity error
subplot(1, 2, 2);
error_vel_norm = sqrt(sum(e_vel.^2, 1));
plot(t_sim, error_vel_norm*1000, 'b-', 'LineWidth', 2.5);
title(sprintf('Total Velocity Error\nMax: %.2f mrad/s, RMS: %.2f mrad/s', ...
              max(error_vel_norm)*1000, sqrt(mean(error_vel_norm.^2))*1000), 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Velocity Error ||ė|| [mrad/s]', 'FontSize', 12);
grid on; grid minor;

sgtitle('Position and Velocity Tracking Errors', 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, sprintf('position_velocity_errors_%s.png', traj)));
close(gcf);

% 5. CONTROL TORQUES
figure('Name', 'Control Torques', 'Position', [100 100 1400 800]);

for joint = 1:7
    subplot(3, 3, joint);
    plot(t_sim, tau_computed(:,joint), 'r-', 'LineWidth', 2);
    title(sprintf('Joint %d Torque - %s', joint, joint_names{joint}), 'FontSize', 12);
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Torque [Nm]', 'FontSize', 10);
    grid on; grid minor;
    
    % Add torque statistics
    max_torque = max(abs(tau_computed(:,joint)));
    rms_torque = sqrt(mean(tau_computed(:,joint).^2));
    text(0.05, 0.95, sprintf('Max: %.1f Nm\nRMS: %.1f Nm', max_torque, rms_torque), ...
         'Units', 'normalized', 'VerticalAlignment', 'top', 'FontSize', 9, ...
         'BackgroundColor', 'white', 'EdgeColor', 'black');
end

% Summary plot with main joints
subplot(3, 3, 8);
plot(t_sim, tau_computed(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Base (J1)');
hold on;
plot(t_sim, tau_computed(:,4), 'b-', 'LineWidth', 2, 'DisplayName', 'Forearm (J4)');
plot(t_sim, tau_computed(:,6), 'g-', 'LineWidth', 2, 'DisplayName', 'Wrist2 (J6)');
title('Main Joint Torques Comparison', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 10);
ylabel('Torque [Nm]', 'FontSize', 10);
legend('show', 'FontSize', 9); grid on;

% Torque norm
subplot(3, 3, 9);
torque_norm = sqrt(sum(tau_computed.^2, 2));
plot(t_sim, torque_norm, 'k-', 'LineWidth', 2.5);
title(sprintf('Total Torque Norm\nMax: %.1f Nm, RMS: %.1f Nm', ...
              max(torque_norm), sqrt(mean(torque_norm.^2))), 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 10);
ylabel('||τ|| [Nm]', 'FontSize', 10);
grid on; grid minor;

sgtitle('Control Torques for All Joints', 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, sprintf('control_torques_%s.png', traj)));
close(gcf);

fprintf('✓ Grafici salvati in: %s\n', strcat(plot_dir, sprintf("_%s", traj)));
fprintf('  - 7 grafici joint tracking: joint_1_tracking.png → joint_7_tracking.png\n');
fprintf('  - End-effector tracking: end_effector_tracking.png\n');
fprintf('  - Errori posizione/velocità: position_velocity_errors.png\n');
fprintf('  - Coppie di controllo: control_torques.png\n');

%% SALVATAGGIO RISULTATI
results = struct();
results.controller = 'FBL_numeric';
results.algorithm = 'recursive_newton_euler';
results.simulation_time = sim_time;
results.max_error = total_max_error;
results.rms_error = total_rms_error;
results.simulation = struct('t', t_sim, 'q', q_sim, 'qd', qd_sim);
results.desired = struct('q', q_desired, 'qd', qd_desired);
results.errors = struct('pos', e_pos, 'vel', e_vel);
results.timestamp = datetime('now');

save(resultsPath, 'results', '-v7.3');

fprintf('\n📁 Risultati salvati: %s\n', resultsPath);
fprintf('🎉 Simulazione completata!\n');

% Comando per calcolare l'errore finale end-effector da file .mat
try
    load(resultsPath, 'results');
    if isfield(results, 'ee_metrics') && isfield(results.ee_metrics, 'final_error_total')
        fprintf('\n=== ERRORE FINALE END-EFFECTOR (da file) ===\n');
        fprintf('Errore finale totale: %.2f mm\n', results.ee_metrics.final_error_total);
        fprintf('  Errore X: %.2f mm\n', results.ee_metrics.final_error_xyz(1));
        fprintf('  Errore Y: %.2f mm\n', results.ee_metrics.final_error_xyz(2));
        fprintf('  Errore Z: %.2f mm\n', results.ee_metrics.final_error_xyz(3));
        fprintf('Posizione finale effettiva: [%.4f, %.4f, %.4f] m\n', results.ee_metrics.final_pos_actual);
        fprintf('Posizione finale target:    [%.4f, %.4f, %.4f] m\n', results.ee_metrics.final_pos_target);
    else
        fprintf('Calcolo dell''errore finale dal file...\n');
        addpath('../lib'); DH = get_DH();
        q_final = results.simulation.q(:, end);
        q_target = results.desired.q(:, end);
        T_actual = eye(4); T_target = eye(4);
        for j = 1:7
            a = DH(j,1); alpha = DH(j,2); d = DH(j,3);
            theta_actual = q_final(j) + DH(j,4); theta_target = q_target(j) + DH(j,4);
            T_i_actual = [cos(theta_actual), -sin(theta_actual)*cos(alpha), sin(theta_actual)*sin(alpha), a*cos(theta_actual); sin(theta_actual), cos(theta_actual)*cos(alpha), -cos(theta_actual)*sin(alpha), a*sin(theta_actual); 0, sin(alpha), cos(alpha), d; 0, 0, 0, 1];
            T_i_target = [cos(theta_target), -sin(theta_target)*cos(alpha), sin(theta_target)*sin(alpha), a*cos(theta_target); sin(theta_target), cos(theta_target)*cos(alpha), -cos(theta_target)*sin(alpha), a*sin(theta_target); 0, sin(alpha), cos(alpha), d; 0, 0, 0, 1];
            T_actual = T_actual * T_i_actual; T_target = T_target * T_i_target;
        end
        pos_actual = T_actual(1:3, 4); pos_target = T_target(1:3, 4);
        error_final = norm(pos_actual - pos_target) * 1000;
        fprintf('\n=== ERRORE FINALE END-EFFECTOR (calcolato) ===\n');
        fprintf('Errore finale totale: %.2f mm\n', error_final);
        fprintf('Posizione finale effettiva: [%.4f, %.4f, %.4f] m\n', pos_actual);
        fprintf('Posizione finale target:    [%.4f, %.4f, %.4f] m\n', pos_target);
    end
catch ME
    fprintf('ERRORE: %s\n', ME.message);
end