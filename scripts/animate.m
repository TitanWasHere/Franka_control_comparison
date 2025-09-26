%% ANIMAZIONE TRAIETTORIA ROBOT DA FILE DI RISULTATI (v8 - OTTIMIZZATO)
clear; clc; close all;

fprintf('=== ANIMAZIONE TRAIETTORIA FBL (VISTA DOPPIA) ===\n\n');

%% 1. CARICAMENTO DATI
fprintf('Caricamento del file di risultati...\n');
try
    %load('FBL_simple_trajectory_full_uncertainty_results.mat', 'results');
    load('FBL_numeric_results_double_more.mat', 'results_numeric');
    %load('PBC_numeric_results.mat', 'results_pbc_numeric');
    fprintf('✓ File caricato con successo.\n');
catch ME
    fprintf('ERRORE: Impossibile trovare il file dei risultati.\n');
    rethrow(ME);
end

results = results_numeric;

t_sim       = results.simulation.t;
q_sim_7dof  = results.simulation.q';
q_desired_7dof = results.desired.q';

%% 2. AUMENTA LA CONFIGURAZIONE A 9 GIUNTI PER COMPATIBILITÀ
gripper_state = [0.02, 0.02];
num_points = size(q_sim_7dof, 1);
q_sim     = [q_sim_7dof, repmat(gripper_state, num_points, 1)];
q_desired = [q_desired_7dof, repmat(gripper_state, num_points, 1)];
fprintf('✓ Dati di traiettoria aumentati a 9 giunti.\n');

%% 3. SETUP MODELLO ROBOT E FIGURA
fprintf('Setup del modello del robot e dell''ambiente di visualizzazione...\n');
try
    robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
catch ME
    fprintf('ERRORE: Assicurati di avere la Robotics System Toolbox™ installata.\n');
    rethrow(ME);
end

fig = figure('Name', 'Animazione Traiettoria FBL - Vista Dettagliata', ...
    'Position', [50 50 1600 800], 'Color', 'w');
sgtitle('Analisi Traiettoria FBL', 'FontSize', 16, 'FontWeight', 'bold');

% --- Subplot 1: Vista con Mesh ---
ax1 = subplot(1, 2, 1);
show(robot, q_sim(1,:), 'Parent', ax1, ...
    'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Vista Realistica (con Mesh)', 'FontSize', 12);
xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]'); zlabel(ax1, 'Z [m]');
view(ax1, 135, 25);

% Imposta vista più lontana
xlim(ax1, [-0.8, 0.8]); 
ylim(ax1, [-0.8, 0.8]);  
zlim(ax1, [-0.3 1.2]);  

% --- Subplot 2: Vista a Scheletro ---
ax2 = subplot(1, 2, 2);
show(robot, q_sim(1,:), 'Parent', ax2, ...
    'PreservePlot', false, 'Frames', 'off', 'Visuals', 'off');
hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
title(ax2, 'Vista Cinematica (Scheletro con Giunti)', 'FontSize', 12);
xlabel(ax2, 'X [m]'); ylabel(ax2, 'Y [m]'); zlabel(ax2, 'Z [m]');
view(ax2, 135, 25);

% Imposta vista più lontana
xlim(ax2, [-0.8, 0.8]);
ylim(ax2, [-0.8, 0.8]);
zlim(ax2, [-0.3 1.2]);

% --- Giunti e link (ignora le dita del gripper) ---
bodies = setdiff(robot.BodyNames, {'panda_leftfinger','panda_rightfinger'});
n_bodies = numel(bodies);

joint_markers = gobjects(n_bodies,1);
link_lines    = gobjects(n_bodies,1);

for b = 1:n_bodies
    T = getTransform(robot, q_sim(1,:), bodies{b});
    p = tform2trvec(T);

    joint_markers(b) = plot3(ax2, p(1), p(2), p(3), 'ko', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 6);

    parentBody = robot.getBody(bodies{b}).Parent;
    if ~strcmp(parentBody.Name, 'base')
        Tp = getTransform(robot, q_sim(1,:), parentBody.Name);
        pp = tform2trvec(Tp);
        link_lines(b) = plot3(ax2, [pp(1) p(1)], [pp(2) p(2)], [pp(3) p(3)], ...
            'k-', 'LineWidth', 1.5);
    end
end

linkprop([ax1, ax2], {'CameraPosition','CameraUpVector','CameraTarget'});
fprintf('✓ Setup visualizzazione completato.\n');

%% 4. POSIZIONI END-EFFECTOR
fprintf('Calcolo delle posizioni dell''end-effector...\n');
ee_name = 'panda_link8';  % CORRETTO: Usa flange invece di panda_hand

n_points = size(q_sim, 1);
pos_desired_ee = zeros(n_points, 3);
pos_actual_ee  = zeros(n_points, 3);

for i = 1:n_points
    T_desired = getTransform(robot, q_desired(i,:), ee_name);
    pos_desired_ee(i,:) = tform2trvec(T_desired);

    T_actual = getTransform(robot, q_sim(i,:), ee_name);
    pos_actual_ee(i,:) = tform2trvec(T_actual);
end
fprintf('✓ Calcolo cinematico completato.\n');

%% 5. TRAIETTORIE
axes_list = [ax1, ax2];
plot_handles = struct();

for i = 1:length(axes_list)
    ax = axes_list(i);
    plot_handles(i).desired = plot3(ax, pos_desired_ee(:,1), pos_desired_ee(:,2), pos_desired_ee(:,3), ...
        '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 2.5);
    plot_handles(i).actual_trail = plot3(ax, NaN, NaN, NaN, '-', ...
        'Color', [0.1 0.4 0.8, 0.7], 'LineWidth', 2);
    plot_handles(i).ee_marker = plot3(ax, pos_actual_ee(1,1), pos_actual_ee(1,2), pos_actual_ee(1,3), 'o', ...
        'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
end

% Nessuna legenda!

%% 6. ANIMAZIONE (OTTIMIZZATA)
fprintf('Avvio animazione... (Premi CTRL+C per interrompere)\n');

% --- IMPOSTA LA RISOLUZIONE DELL'ANIMAZIONE ---
animation_fps = 30; % Fotogrammi al secondo desiderati
simulation_dt = mean(diff(t_sim)); % Calcola il passo temporale medio della simulazione
% Calcola quanti punti saltare per raggiungere il target FPS
step = max(1, round((1/animation_fps) / simulation_dt));
% ------------------------------------------------

speedup = 2;   % Fattore di velocità (2 = doppia velocità)
animation_timer = tic;

% Usa 'step' per saltare i frame e ridurre il carico di rendering
for i = 1:step:length(t_sim)
    current_sim_time = t_sim(i);
    
    % Aggiorna il robot con la mesh (l'operazione più pesante)
    show(robot, q_sim(i,:), 'Parent', ax1, 'PreservePlot', false, ...
        'Frames','off','Visuals','on');
    
    % Aggiorna le traiettorie e il marker dell'end-effector (molto veloce)
    for ax_idx = 1:2
        set(plot_handles(ax_idx).actual_trail, ...
            'XData', pos_actual_ee(1:i,1), ...
            'YData', pos_actual_ee(1:i,2), ...
            'ZData', pos_actual_ee(1:i,3));
        set(plot_handles(ax_idx).ee_marker, ...
            'XData', pos_actual_ee(i,1), ...
            'YData', pos_actual_ee(i,2), ...
            'ZData', pos_actual_ee(i,3));
    end
    
    % Aggiorna lo scheletro (molto veloce)
    for b = 1:n_bodies
        T = getTransform(robot, q_sim(i,:), bodies{b});
        p = tform2trvec(T);
        set(joint_markers(b), 'XData', p(1), 'YData', p(2), 'ZData', p(3));

        parentBody = robot.getBody(bodies{b}).Parent;
        if ~strcmp(parentBody.Name, 'base')
            Tp = getTransform(robot, q_sim(i,:), parentBody.Name);
            pp = tform2trvec(Tp);
            set(link_lines(b), ...
                'XData', [pp(1) p(1)], ...
                'YData', [pp(2) p(2)], ...
                'ZData', [pp(3) p(3)]);
        end
    end
    
    % Aggiorna il titolo
    sgtitle(sprintf('Analisi Traiettoria FBL - Tempo: %.2f s / %.2f s', ...
        current_sim_time, t_sim(end)));
    
    % Disegna le modifiche
    drawnow;
    
    % Controlla la velocità di riproduzione
    elapsed_real_time = toc(animation_timer);
    time_to_wait = (current_sim_time/speedup) - elapsed_real_time;
    %if time_to_wait > 0
    %    pause(time_to_wait);
    %end
end

sgtitle(sprintf('Animazione Completata (Tempo Finale: %.2f s)', t_sim(end)), ...
    'Color', [0 .6 0]);
fprintf('\n🎉 Animazione completata!\n');