%% SIMPLE TRAJECTORY USING FEEDBACK LINEARIZATION - PURE NUMERIC VERSION

clear; clc; close all;

fprintf('=== SIMPLE TRAJECTORY - FBL PURE NUMERIC ===\n\n');
fprintf('Questa versione usa algoritmi numerici puri (Newton-Euler)\n');
fprintf('invece delle funzioni simboliche lente generate da MATLAB.\n\n');

%% Load numerical parameters
addpath("../lib");
run("../lib/setup_numerical_parameters.m");

controller_gains = setup_controller_gains();

filename = '../data/FBL_numeric_results_double_more.mat';
controller_gains.Kp = controller_gains.Kp * 2;
controller_gains.Kd = controller_gains.Kd + 10;

fprintf('Gains FBL numerici:\n');
fprintf('  Kp = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kp));
fprintf('  Kd = diag([%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f])\n', diag(controller_gains.Kd));

%% TRAJECTORY DEFINITION
q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]'; 
qf = q0 + [0.3, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3]';

limits = setup_robot_limits();
q_max_safe = limits.q_max - limits.safety_margin;
q_min_safe = limits.q_min + limits.safety_margin;
qf = max(q_min_safe, min(qf, q_max_safe));

%% SETUP SIMULATION
T_final = 8.0;
x0 = [q0; zeros(7,1)];

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'MaxStep', 0.01, ...
                 'Stats', 'off', 'InitialStep', 1e-4, 'Refine', 1);

uncertain_params = setup_uncertainty_levels(true_params_vec);
uncertain_params = uncertain_params.inertia_coriolis; 

%% SIMULAZIONE CON ALGORITMI NUMERICI PURI

fprintf('\n=== SIMULAZIONE FBL NUMERICA PURA ===\n');
fprintf('Usando algoritmi Newton-Euler ricorsivi O(n)...\n');

tic;
[t_sim, x_sim] = ode45(@(t, x) robot_dynamics_numeric(t, x, q0, qf, T_final, @FBL_numeric, ...
                                                      controller_gains, true_params_vec, uncertain_params), ...
                       [0 T_final], x0, options);
sim_time = toc;

fprintf('✓ Simulazione numerica completata in %.3f secondi\n', sim_time);
fprintf('  Punti simulati: %d\n', length(t_sim));
fprintf('  Frequenza media: %.1f Hz\n', length(t_sim)/T_final);

%% ESTRAZIONE DATI E ANALISI
fprintf('\n=== ANALISI RISULTATI NUMERICI ===\n');

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

%% GRAFICI OTTIMIZZATI
fprintf('\n=== GENERAZIONE GRAFICI ===\n');

figure('Name', 'FBL Numerico - Risultati', 'Position', [100 100 1600 900]);

% Grafico 1: Errore totale nel tempo
subplot(2, 4, 1);
error_norm = sqrt(sum(e_pos.^2, 1));
plot(t_sim, error_norm*1000, 'r-', 'LineWidth', 2);
title('Errore Totale di Posizione');
xlabel('Tempo [s]');
ylabel('||e|| [mrad]');
grid on;

% Grafico 2: Errori per giunto
subplot(2, 4, 2);
bar(1:7, max_pos_error*1000, 'FaceColor', [0.2 0.6 0.8]);
title('Errori Massimi per Giunto');
xlabel('Numero Giunto');
ylabel('Errore [mrad]');
grid on;

% Grafico 3: Errore End-Effector nel Tempo
subplot(2, 4, 3);

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

plot(t_sim, ee_error, 'r-', 'LineWidth', 2);
title(sprintf('Errore End-Effector\nMax: %.2f mm, RMS: %.2f mm', max_ee_error, rms_ee_error));
xlabel('Tempo [s]');
ylabel('Errore [mm]');
grid on;

fprintf('✓ Errore end-effector - Max: %.2f mm, RMS: %.2f mm\n', max_ee_error, rms_ee_error);

% Grafico 4: Spettro di frequenza dell'errore
subplot(2, 4, 4);
dt = mean(diff(t_sim));
fs = 1/dt;
N = length(error_norm);
f = (0:N-1)*(fs/N);
Y = fft(error_norm);
P = abs(Y/N).^2;
plot(f(1:floor(N/2)), P(1:floor(N/2)), 'b-', 'LineWidth', 1.5);
title('Spettro Errore di Posizione');
xlabel('Frequenza [Hz]');
ylabel('Densità Spettrale');
grid on;
xlim([0 min(10, fs/2)]);

% Grafico 5: Coppie di controllo
subplot(2, 4, 5);
tau_computed = zeros(length(t_sim), 7);
for i = 1:length(t_sim)
    [q_d, qd_d, qdd_d] = generate_trajectory(t_sim(i), q0, qf, T_final);
    tau_computed(i,:) = FBL_numeric(q_sim(:,i), qd_sim(:,i), q_d, qd_d, qdd_d, controller_gains, uncertain_params);
end
plot(t_sim, tau_computed(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 1 (Base)');
hold on;
plot(t_sim, tau_computed(:,4), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 4 (Gomito)');
plot(t_sim, tau_computed(:,6), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 6 (Polso)');
title('Coppie di Controllo');
xlabel('Tempo [s]');
ylabel('Coppia [Nm]');
legend('show');
grid on;

% Grafico 6: Velocità
subplot(2, 4, 6);
vel_norm = sqrt(sum(qd_sim.^2, 1));
vel_des_norm = sqrt(sum(qd_desired.^2, 1));
plot(t_sim, vel_des_norm, 'g--', 'LineWidth', 2, 'DisplayName', 'Desiderata');
hold on;
plot(t_sim, vel_norm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Effettiva');
title('Velocità Totale');
xlabel('Tempo [s]');
ylabel('||q̇|| [rad/s]');
legend('show');
grid on;

% Grafico 7: Errori per giunto nel tempo
subplot(2, 4, 7);
plot(t_sim, abs(e_pos(1,:))*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 1 (Base)');
hold on;
plot(t_sim, abs(e_pos(4,:))*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 4 (Gomito)');
plot(t_sim, abs(e_pos(6,:))*1000, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Giunto 6 (Polso)');
title('Errori Principali nel Tempo');
xlabel('Tempo [s]');
ylabel('|Errore| [mrad]');
legend('show');
grid on;

% Grafico 8: Fase portrait (errore vs velocità errore)
subplot(2, 4, 8);
e_vel_norm = sqrt(sum(e_vel.^2, 1));
plot(error_norm*1000, e_vel_norm*1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(error_norm(1)*1000, e_vel_norm(1)*1000, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Inizio');
plot(error_norm(end)*1000, e_vel_norm(end)*1000, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Fine');
title('Phase Portrait');
xlabel('Errore Posizione [mrad]');
ylabel('Errore Velocità [mrad/s]');
legend('show');
grid on;

sgtitle('FBL Controller - Algoritmi Numerici con Parametri Incerti', 'FontSize', 16, 'FontWeight', 'bold');

%% SALVATAGGIO RISULTATI
results_numeric = struct();
results_numeric.controller = 'FBL_numeric';
results_numeric.algorithm = 'recursive_newton_euler';
results_numeric.simulation_time = sim_time;
results_numeric.max_error = total_max_error;
results_numeric.rms_error = total_rms_error;
results_numeric.simulation = struct('t', t_sim, 'q', q_sim, 'qd', qd_sim);
results_numeric.desired = struct('q', q_desired, 'qd', qd_desired);
results_numeric.errors = struct('pos', e_pos, 'vel', e_vel);
results_numeric.timestamp = datetime('now');

save(filename, 'results_numeric', '-v7.3');

fprintf('\n📁 Risultati salvati: %s\n', filename);
fprintf('🎉 Simulazione completata!\n');

% Comando per calcolare l'errore finale end-effector da file .mat
try
    load(filename, 'results_numeric');
    if isfield(results_numeric, 'ee_metrics') && isfield(results_numeric.ee_metrics, 'final_error_total')
        fprintf('\n=== ERRORE FINALE END-EFFECTOR (da file) ===\n');
        fprintf('Errore finale totale: %.2f mm\n', results_numeric.ee_metrics.final_error_total);
        fprintf('  Errore X: %.2f mm\n', results_numeric.ee_metrics.final_error_xyz(1));
        fprintf('  Errore Y: %.2f mm\n', results_numeric.ee_metrics.final_error_xyz(2));
        fprintf('  Errore Z: %.2f mm\n', results_numeric.ee_metrics.final_error_xyz(3));
        fprintf('Posizione finale effettiva: [%.4f, %.4f, %.4f] m\n', results_numeric.ee_metrics.final_pos_actual);
        fprintf('Posizione finale target:    [%.4f, %.4f, %.4f] m\n', results_numeric.ee_metrics.final_pos_target);
    else
        fprintf('Calcolo dell''errore finale dal file...\n');
        addpath('../lib'); DH = get_DH();
        q_final = results_numeric.simulation.q(:, end);
        q_target = results_numeric.desired.q(:, end);
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