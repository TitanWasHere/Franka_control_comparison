%% SIMPLE TRAJECTORY USING BANG-COAST-BANG CONTROL PROFILE - FBL NUMERIC

clear; clc; close all;

fprintf('=== TRAIETTORIA BANG-COAST-BANG ===\n\n');
fprintf('Questa versione usa un profilo di accelerazione bang-coast-bang:\n');
fprintf('- Prima fase: accelerazione costante (bang-up)\n');
fprintf('- Seconda fase: velocità costante (coast)\n');
fprintf('- Terza fase: decelerazione costante (bang-down)\n');
fprintf('- Rest-to-rest: velocità zero agli estremi\n\n');

%% CONFIGURAZIONE TRAIETTORIA
% Frazione di tempo in coast (regolabile)
COAST_FRACTION = 0.4;  % 40% del tempo in velocità costante

fprintf('CONFIGURAZIONE BANG-COAST-BANG:\n');
fprintf('  Frazione coast: %.1f%% del tempo totale\n', COAST_FRACTION*100);
fprintf('  Frazione accelerazione: %.1f%% del tempo totale\n', (1-COAST_FRACTION)/2*100);
fprintf('  Frazione decelerazione: %.1f%% del tempo totale\n', (1-COAST_FRACTION)/2*100);

%% SETUP LIBRERIE E PARAMETRI
addpath("../../lib");
addpath(".");
run("../../lib/setup_numerical_parameters.m");

controller_gains = setup_controller_gains();

% Configurazione controller gains
controller_gains.Kp = controller_gains.Kp * 2;
controller_gains.Kd = controller_gains.Kd + 10;

fprintf('\nGains FBL numerici:\n');
fprintf('  Kp = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kp));
fprintf('  Kd = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kd));

%% DEFINIZIONE TRAIETTORIA BANG-COAST-BANG
TRAJ_A=true;

if TRAJ_A
    q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]'; 
    qf = q0 + [0.3, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3]';
else
    q0 = [3*pi/2, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]';
    qf = q0 + [0.5, -0.5, pi/2, -pi/2, 0.0,pi, 0.5]';
end

% Applica limiti di sicurezza
limits = setup_robot_limits();
q_max_safe = limits.q_max - limits.safety_margin;
q_min_safe = limits.q_min + limits.safety_margin;
qf = max(q_min_safe, min(qf, q_max_safe));

% Calcola spostamenti per ogni joint
Delta_q = qf - q0;

fprintf('\nPARAMETRI TRAIETTORIA BANG-COAST-BANG:\n');
fprintf('  Configurazione iniziale q0: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad\n', q0);
fprintf('  Configurazione finale qf:   [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad\n', qf);
fprintf('  Spostamenti Delta_q:        [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad\n', Delta_q);
fprintf('  Spostamenti Delta_q (gradi): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]°\n', rad2deg(Delta_q));

%% SETUP SIMULAZIONE
T_final = 8.0;  
METCHED_START = true;
if METCHED_START
    q0_actual = q0;
else
    q0_actual = q0 + deg2rad([5,-10,0,8,0,0,0]');
end
x0 = [q0_actual; zeros(7,1)];

% Calcolo tempi delle fasi
T_accel = T_final * (1 - COAST_FRACTION) / 2;
T_coast = T_final * COAST_FRACTION;
T_decel = T_accel;

fprintf('\nPARAMETRI TEMPORALI:\n');
fprintf('  Tempo totale: %.1f s\n', T_final);
fprintf('  Tempo accelerazione: %.1f s (0 - %.1f s)\n', T_accel, T_accel);
fprintf('  Tempo coast: %.1f s (%.1f - %.1f s)\n', T_coast, T_accel, T_accel + T_coast);
fprintf('  Tempo decelerazione: %.1f s (%.1f - %.1f s)\n', T_decel, T_accel + T_coast, T_final);

% Calcola parametri caratteristici della traiettoria bang-coast-bang
accelerations = Delta_q ./ (T_accel * (T_accel + T_coast));
velocities_coast = accelerations * T_accel;

fprintf('\nCARATTERISTICHE DINAMICHE:\n');
fprintf('  Accelerazioni: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s²\n', accelerations);
fprintf('  Velocità coast: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', velocities_coast);

% Opzioni integrazione numerica
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'MaxStep', 0.01, ...
                 'Stats', 'off', 'InitialStep', 1e-4, 'Refine', 1);

% Parametri incertezza
uncertain_params = setup_uncertainty_levels(true_params_vec);
uncertain_params = uncertain_params.inertia_coriolis; 

%% SIMULAZIONE TRAIETTORIA BANG-COAST-BANG
fprintf('\n=== SIMULAZIONE FBL CON TRAIETTORIA BANG-COAST-BANG ===\n');
fprintf('Integrando equazioni dinamiche...\n');

tic;
[t_sim, x_sim] = ode45(@(t, x) robot_dynamics_bang_coast_bang(t, x, q0, qf, T_final, @FBL_numeric, ...
                                                              controller_gains, true_params_vec, uncertain_params, COAST_FRACTION), ...
                       [0 T_final], x0, options);
sim_time = toc;

fprintf('✓ Simulazione completata in %.3f secondi\n', sim_time);
fprintf('  Punti simulati: %d\n', length(t_sim));
fprintf('  Frequenza media: %.1f Hz\n', length(t_sim)/T_final);
fprintf('  Tempo simulazione/tempo reale: %.2fx\n', sim_time/T_final);

%% ANALISI RISULTATI
fprintf('\n=== ANALISI PRESTAZIONI BANG-COAST-BANG ===\n');

% Estrai dati simulazione
q_sim = x_sim(:, 1:7)';        % Posizioni: 7 x N_points
qd_sim = x_sim(:, 8:14)';      % Velocità: 7 x N_points
n_points = length(t_sim);

% Genera traiettoria desiderata per analisi
q_desired = zeros(7, n_points);
qd_desired = zeros(7, n_points);
qdd_desired = zeros(7, n_points);

fprintf('Calcolo traiettoria di riferimento bang-coast-bang...\n');
for i = 1:n_points
    [q_desired(:,i), qd_desired(:,i), qdd_desired(:,i)] = ...
        generate_bang_coast_bang_trajectory(t_sim(i), q0, qf, T_final, COAST_FRACTION);
end

% Calcola errori di tracking
e_pos = q_sim - q_desired;      % Errore posizione
e_vel = qd_sim - qd_desired;    % Errore velocità

% Metriche di performance globali
max_pos_error = max(abs(e_pos), [], 2);                    % Errore max per joint
rms_pos_error = sqrt(mean(e_pos.^2, 2));                  % Errore RMS per joint
total_max_error = max(max_pos_error);                      % Errore max assoluto
total_rms_error = sqrt(mean(sum(e_pos.^2, 1)));          % Errore RMS totale

fprintf('PERFORMANCE TRACKING:\n');
fprintf('  Errore max assoluto: %.4f rad (%.2f°)\n', total_max_error, rad2deg(total_max_error));
fprintf('  Errore RMS totale: %.4f rad (%.2f°)\n', total_rms_error, rad2deg(total_rms_error));
fprintf('  Errore max per joint: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]° \n', ...
        rad2deg(max_pos_error'));

% Verifica caratteristiche bang-coast-bang nelle diverse fasi
accel_end_idx = find(t_sim >= T_accel, 1);
coast_end_idx = find(t_sim >= T_accel + T_coast, 1);

if ~isempty(accel_end_idx) && ~isempty(coast_end_idx)
    fprintf('\nCARATTERISTICHE BANG-COAST-BANG:\n');
    fprintf('  Fine accelerazione (t=%.1fs):\n', t_sim(accel_end_idx));
    fprintf('    Velocità simulate: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', qd_sim(:, accel_end_idx));
    fprintf('    Velocità teoriche: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s\n', velocities_coast);
    
    % Verifica velocità costante durante coast
    coast_start_idx = accel_end_idx;
    coast_velocities = qd_sim(:, coast_start_idx:coast_end_idx);
    coast_std = std(coast_velocities, 0, 2);
    fprintf('  Durante coast:\n');
    fprintf('    Deviazione standard velocità: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad/s\n', coast_std);
end

%% ANALISI END-EFFECTOR
fprintf('\n=== ANALISI END-EFFECTOR ===\n');

DH = get_DH();

% Calcola posizioni end-effector
ee_pos_sim = zeros(3, n_points);
ee_pos_des = zeros(3, n_points);

fprintf('Calcolo traiettorie end-effector...\n');

for i = 1:n_points
    % Posizione effettiva
    T_sim = eye(4);
    for j = 1:7
        a = DH(j, 1); alpha = DH(j, 2); d = DH(j, 3); 
        theta = q_sim(j,i) + DH(j, 4);
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
        a = DH(j, 1); alpha = DH(j, 2); d = DH(j, 3); 
        theta = q_desired(j,i) + DH(j, 4);
        T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1];
        T_des = T_des * T_i;
    end
    ee_pos_des(:,i) = T_des(1:3, 4);
end

% Metriche end-effector
ee_error = sqrt(sum((ee_pos_sim - ee_pos_des).^2, 1)) * 1000;  % [mm]
max_ee_error = max(ee_error);
rms_ee_error = sqrt(mean(ee_error.^2));
final_ee_error = ee_error(end);

fprintf('PERFORMANCE END-EFFECTOR:\n');
fprintf('  Errore tracking - Max: %.2f mm, RMS: %.2f mm\n', max_ee_error, rms_ee_error);
fprintf('  Errore finale: %.2f mm\n', final_ee_error);

%% SETUP DIRECTORY PLOTS
plot_dir = '../plots/bang_coast_bang';
if ~exist(plot_dir, 'dir')
    mkdir(plot_dir);
end

%% CALCOLO COPPIE DI CONTROLLO
fprintf('Calcolo coppie di controllo...\n');
tau_sample_idx = 1:max(1, floor(length(t_sim)/200)):length(t_sim);
tau_computed = zeros(length(tau_sample_idx), 7);

for i = 1:length(tau_sample_idx)
    idx = tau_sample_idx(i);
    [q_d, qd_d, qdd_d] = generate_bang_coast_bang_trajectory(t_sim(idx), q0, qf, T_final, COAST_FRACTION);
    tau_computed(i,:) = FBL_numeric(q_sim(:,idx), qd_sim(:,idx), q_d, qd_d, qdd_d, controller_gains, uncertain_params);
end

%% GENERAZIONE GRAFICI E SALVATAGGIO
fprintf('\n=== GENERAZIONE E SALVATAGGIO GRAFICI ===\n');

% 1. TRACKING JOINT POSITIONS (7 separate plots)
joint_names = {'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)', 'Forearm (J4)', ...
               'Wrist1 (J5)', 'Wrist2 (J6)', 'Wrist3 (J7)'};

for joint = 1:7
    figure('Name', sprintf('Joint %d Bang-Coast-Bang Tracking', joint), 'Position', [100 100 800 600]);
    plot(t_sim, rad2deg(q_desired(joint,:)), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired');
    hold on;
    plot(t_sim, rad2deg(q_sim(joint,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    % Add transition lines
    xline(T_accel, 'k:', 'LineWidth', 2, 'DisplayName', 'Accel→Coast');
    xline(T_accel + T_coast, 'k--', 'LineWidth', 2, 'DisplayName', 'Coast→Decel');
    title(sprintf('Joint %d Bang-Coast-Bang Position Tracking - %s', joint, joint_names{joint}), 'FontSize', 14);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Angle [deg]', 'FontSize', 12);
    legend('show', 'FontSize', 12, 'Location', 'best');
    grid on; grid minor;
    
    % Add error info and bang-coast-bang parameters
    joint_error_max = max_pos_error(joint);
    joint_error_rms = rms_pos_error(joint);
    joint_accel = abs(accelerations(joint));
    joint_coast_vel = abs(velocities_coast(joint));
    
    annotation('textbox', [0.15, 0.7, 0.35, 0.25], 'String', ...
               sprintf('Bang-Coast-Bang Profile:\nAccel: %.2f rad/s²\nCoast Vel: %.2f rad/s\nCoast: %.0f%% of time\n\nTracking Error:\nMax: %.2f°\nRMS: %.2f°', ...
                      joint_accel, joint_coast_vel, COAST_FRACTION*100, rad2deg(joint_error_max), rad2deg(joint_error_rms)), ...
               'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    saveas(gcf, fullfile(plot_dir, sprintf('joint_%d_tracking.png', joint)));
    close(gcf);
end

% 2. END-EFFECTOR TRACKING
figure('Name', 'End-Effector Bang-Coast-Bang Tracking', 'Position', [100 100 1200 800]);

% 3D trajectory
subplot(2, 2, 1);
plot3(ee_pos_des(1,:), ee_pos_des(2,:), ee_pos_des(3,:), 'g--', 'LineWidth', 3, 'DisplayName', 'Desired');
hold on;
plot3(ee_pos_sim(1,:), ee_pos_sim(2,:), ee_pos_sim(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
plot3(ee_pos_sim(1,1), ee_pos_sim(2,1), ee_pos_sim(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(ee_pos_sim(1,end), ee_pos_sim(2,end), ee_pos_sim(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
title('End-Effector 3D Trajectory (Bang-Coast-Bang)', 'FontSize', 14);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('show', 'Location', 'best'); grid on; axis equal;

% X, Y, Z components with transition lines
subplot(2, 2, 2);
plot(t_sim, ee_pos_des(1,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired X');
hold on;
plot(t_sim, ee_pos_sim(1,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual X');
xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
title('End-Effector X Position');
xlabel('Time [s]'); ylabel('X [m]'); legend('show'); grid on;

subplot(2, 2, 3);
plot(t_sim, ee_pos_des(2,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired Y');
hold on;
plot(t_sim, ee_pos_sim(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Y');
xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
title('End-Effector Y Position');
xlabel('Time [s]'); ylabel('Y [m]'); legend('show'); grid on;

subplot(2, 2, 4);
plot(t_sim, ee_pos_des(3,:), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired Z');
hold on;
plot(t_sim, ee_pos_sim(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Z');
xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
title('End-Effector Z Position');
xlabel('Time [s]'); ylabel('Z [m]'); legend('show'); grid on;

sgtitle(sprintf('End-Effector Bang-Coast-Bang Tracking (Max Error: %.2f mm, RMS: %.2f mm)', max_ee_error, rms_ee_error), 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, 'end_effector_tracking.png'));
close(gcf);

% 3. VELOCITY TRACKING (BANG-COAST-BANG)
figure('Name', 'Bang-Coast-Bang Velocity Tracking', 'Position', [100 100 1400 1000]);

for joint = 1:7
    subplot(3, 3, joint);
    plot(t_sim, rad2deg(qd_desired(joint,:)), 'g--', 'LineWidth', 2.5, 'DisplayName', 'Desired');
    hold on;
    plot(t_sim, rad2deg(qd_sim(joint,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    
    % Add bang-coast-bang phase lines
    xline(T_accel, 'r:', 'LineWidth', 2, 'DisplayName', 'Accel→Coast');
    xline(T_accel + T_coast, 'r--', 'LineWidth', 2, 'DisplayName', 'Coast→Decel');
    
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

sgtitle(sprintf('Bang-Coast-Bang Velocity Tracking (Max Total Error: %.2f°/s, RMS Total Error: %.2f°/s)', ...
        rad2deg(max_total_vel_error), rad2deg(rms_total_vel_error)), 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, 'velocity_tracking.png'));
close(gcf);

% 4. POSITION AND VELOCITY ERRORS
figure('Name', 'Bang-Coast-Bang Position and Velocity Errors', 'Position', [100 100 1200 600]);

% Total position error
subplot(1, 2, 1);
error_norm = sqrt(sum(e_pos.^2, 1));
plot(t_sim, error_norm*1000, 'r-', 'LineWidth', 2.5);
hold on;
xline(T_accel, 'k:', 'LineWidth', 2, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 2, 'DisplayName', 'Coast→Decel');
title(sprintf('Total Position Error (Bang-Coast-Bang)\nMax: %.2f mrad, RMS: %.2f mrad', ...
              max(error_norm)*1000, sqrt(mean(error_norm.^2))*1000), 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Position Error ||e|| [mrad]', 'FontSize', 12);
legend('show'); grid on; grid minor;

% Total velocity error
subplot(1, 2, 2);
error_vel_norm = sqrt(sum(e_vel.^2, 1));
plot(t_sim, error_vel_norm*1000, 'b-', 'LineWidth', 2.5);
hold on;
xline(T_accel, 'k:', 'LineWidth', 2, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 2, 'DisplayName', 'Coast→Decel');
title(sprintf('Total Velocity Error (Bang-Coast-Bang)\nMax: %.2f mrad/s, RMS: %.2f mrad/s', ...
              max(error_vel_norm)*1000, sqrt(mean(error_vel_norm.^2))*1000), 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Velocity Error ||ė|| [mrad/s]', 'FontSize', 12);
legend('show'); grid on; grid minor;

sgtitle('Bang-Coast-Bang Position and Velocity Tracking Errors', 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, 'position_velocity_errors.png'));
close(gcf);

% 5. CONTROL TORQUES
figure('Name', 'Bang-Coast-Bang Control Torques', 'Position', [100 100 1400 800]);

for joint = 1:7
    subplot(3, 3, joint);
    plot(t_sim(tau_sample_idx), tau_computed(:,joint), 'r-', 'LineWidth', 2);
    hold on;
    xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
    xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
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
plot(t_sim(tau_sample_idx), tau_computed(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Base (J1)');
hold on;
plot(t_sim(tau_sample_idx), tau_computed(:,4), 'b-', 'LineWidth', 2, 'DisplayName', 'Forearm (J4)');
plot(t_sim(tau_sample_idx), tau_computed(:,6), 'g-', 'LineWidth', 2, 'DisplayName', 'Wrist2 (J6)');
xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
title('Main Joint Torques (Bang-Coast-Bang)', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 10);
ylabel('Torque [Nm]', 'FontSize', 10);
legend('show', 'FontSize', 9); grid on;

% Torque norm
subplot(3, 3, 9);
torque_norm = sqrt(sum(tau_computed.^2, 2));
plot(t_sim(tau_sample_idx), torque_norm, 'k-', 'LineWidth', 2.5);
hold on;
xline(T_accel, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Accel→Coast');
xline(T_accel + T_coast, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Coast→Decel');
title(sprintf('Total Torque Norm (Bang-Coast-Bang)\nMax: %.1f Nm, RMS: %.1f Nm', ...
              max(torque_norm), sqrt(mean(torque_norm.^2))), 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 10);
ylabel('||τ|| [Nm]', 'FontSize', 10);
legend('show'); grid on; grid minor;

sgtitle('Bang-Coast-Bang Control Torques for All Joints', 'FontSize', 16);
saveas(gcf, fullfile(plot_dir, 'control_torques.png'));
close(gcf);

fprintf('✓ Grafici Bang-Coast-Bang salvati in: %s\n', plot_dir);
fprintf('  - 7 grafici joint tracking: joint_1_tracking.png → joint_7_tracking.png\n');
fprintf('  - End-effector tracking: end_effector_tracking.png\n');
fprintf('  - Errori posizione/velocità: position_velocity_errors.png\n');
fprintf('  - Coppie di controllo: control_torques.png\n');

%% SALVATAGGIO RISULTATI
filename = '../results/FBL_bang_coast_bang_results.mat';

results = struct();
results.controller = 'FBL_numeric_bang_coast_bang';
results.trajectory_type = 'bang_coast_bang';
results.simulation_time = sim_time;

% Parametri traiettoria
results.trajectory_params = struct();
results.trajectory_params.q0 = q0;
results.trajectory_params.qf = qf;
results.trajectory_params.T_final = T_final;
results.trajectory_params.coast_fraction = COAST_FRACTION;
results.trajectory_params.T_accel = T_accel;
results.trajectory_params.T_coast = T_coast;
results.trajectory_params.T_decel = T_decel;
results.trajectory_params.accelerations = accelerations;
results.trajectory_params.coast_velocities = velocities_coast;

% Performance joint-space
results.performance_joint = struct();
results.performance_joint.max_error = total_max_error;
results.performance_joint.rms_error = total_rms_error;
results.performance_joint.max_error_per_joint = max_pos_error;
results.performance_joint.rms_error_per_joint = rms_pos_error;

% Performance end-effector
results.performance_ee = struct();
results.performance_ee.max_error_mm = max_ee_error;
results.performance_ee.rms_error_mm = rms_ee_error;
results.performance_ee.final_error_mm = final_ee_error;

% Dati simulazione
results.simulation = struct();
results.simulation.t = t_sim;
results.simulation.q = q_sim;
results.simulation.qd = qd_sim;
results.simulation.ee_pos = ee_pos_sim;

% Dati traiettoria desiderata
results.desired = struct();
results.desired.q = q_desired;
results.desired.qd = qd_desired;
results.desired.qdd = qdd_desired;
results.desired.ee_pos = ee_pos_des;

% Errori
results.errors = struct();
results.errors.pos = e_pos;
results.errors.vel = e_vel;
results.errors.ee_mm = ee_error;

% Controller e parametri
results.controller_gains = controller_gains;
results.timestamp = datetime('now');

save(filename, 'results', '-v7.3');

fprintf('\n✓ Risultati salvati: %s\n', filename);

%% SUMMARY FINALE
fprintf(['\n' repmat('=', 1, 70) '\n']);
fprintf('SUMMARY TRAIETTORIA BANG-COAST-BANG\n');
fprintf([repmat('=', 1, 70) '\n']);
fprintf('Traiettoria:\n');
fprintf('  Tipo: Bang-Coast-Bang (rest-to-rest)\n');
fprintf('  Tempo: %.1f s (Accel: %.1fs, Coast: %.1fs, Decel: %.1fs)\n', T_final, T_accel, T_coast, T_decel);
fprintf('  Coast fraction: %.0f%% del tempo totale\n', COAST_FRACTION*100);
fprintf('  Spostamento max: %.2f° (Joint %d)\n', max(rad2deg(abs(Delta_q))), find(abs(Delta_q)==max(abs(Delta_q)), 1));
fprintf('\nPerformance Joint:\n');
fprintf('  Errore max: %.2f° (%.3f rad)\n', rad2deg(total_max_error), total_max_error);
fprintf('  Errore RMS: %.2f° (%.3f rad)\n', rad2deg(total_rms_error), total_rms_error);
fprintf('\nPerformance End-Effector:\n');
fprintf('  Errore tracking max: %.2f mm\n', max_ee_error);
fprintf('  Errore tracking RMS: %.2f mm\n', rms_ee_error);
fprintf('  Errore finale: %.2f mm\n', final_ee_error);
fprintf('\nCaratteristiche Bang-Coast-Bang:\n');
fprintf('  Accelerazione max: %.3f rad/s² (Joint %d)\n', max(abs(accelerations)), find(abs(accelerations)==max(abs(accelerations)), 1));
fprintf('  Velocità coast max: %.3f rad/s (Joint %d)\n', max(abs(velocities_coast)), find(abs(velocities_coast)==max(abs(velocities_coast)), 1));
if exist('coast_std', 'var')
    fprintf('  Deviazione std velocità coast: %.4f rad/s (max)\n', max(coast_std));
end
fprintf('\nComputazione:\n');
fprintf('  Tempo simulazione: %.3f s (%.2fx realtime)\n', sim_time, sim_time/T_final);
fprintf('  Frequenza simulazione: %.1f Hz\n', length(t_sim)/T_final);
fprintf('\nFile salvato: %s\n', filename);
fprintf([repmat('=', 1, 70) '\n']);

fprintf('\n🎯 Simulazione traiettoria bang-coast-bang completata con successo!\n');