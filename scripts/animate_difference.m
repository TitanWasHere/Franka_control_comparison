clear; clc; close all;

fprintf('=== ANIMAZIONE COMPARAZIONE TRAIETTORIE (FBL vs PBC) ===\n\n');

%% config params
CONTROLLER_A = "FBL";
CONTROLLER_B = "PBC";
TRAJ = 'C';
MATCHED_START = true;
OPTIMIZED_GAINS = true;
PROFILE = "bang_bang"; % "quintic", "bang_bang", "bang_coast_bang"
UNCERTAINTY_LEVEL = "extreme"; % Options: "extreme", "high"

%% path setup helper function
get_path = @(controller_name) ...
    local_get_path(controller_name, TRAJ, MATCHED_START, OPTIMIZED_GAINS, PROFILE, UNCERTAINTY_LEVEL);

function path = local_get_path(controller_name, traj_label, matched, optimized, profile, uncertainty)
    try
        [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
        [project_root, ~, ~] = fileparts(script_dir);
    catch
        project_root = pwd; 
    end

    sub_path = sprintf("%s/", controller_name);
    sub_path = strcat(sub_path, sprintf("%s/", uncertainty));
    if matched, sub_path = strcat(sub_path, "matched/"); else, sub_path = strcat(sub_path, "mismatched/"); end
    if optimized, sub_path = strcat(sub_path, "optimized/"); else, sub_path = strcat(sub_path, "unoptimized/"); end
    sub_path = strcat(sub_path, sprintf("%s/", profile));
    
    switch profile
        case "quintic", traj_suffix = "quintic";
        case "bang_bang", traj_suffix = "bb";
        case "bang_coast_bang", traj_suffix = "bcb";
        otherwise, error('Unknown PROFILE type.');
    end
    file_name = sprintf("%s_%s_%s.mat", controller_name, traj_suffix, traj_label);
    path = fullfile(project_root, 'results', sub_path, file_name);
end


%% data loading
controllers_to_compare = struct('name', {CONTROLLER_A, CONTROLLER_B}, ...
    'results', cell(1, 2), ...
    'q_sim', cell(1, 2), ...
    'q_desired', cell(1, 2));

for c = 1:numel(controllers_to_compare)
    controller_name = controllers_to_compare(c).name;
    path = get_path(controller_name);
    fprintf('Loading results file for %s from ''%s''...\n', controller_name, path);
    try
        data = load(path, 'results');
        controllers_to_compare(c).results = data.results;
        fprintf('File loaded succesfully.\n');
    catch ME
        fprintf('Could not find the results file for %s.\n', controller_name);
        rethrow(ME);
    end
end

% --- Determine consistent length across all datasets ---
t_sim = controllers_to_compare(1).results.simulation.t;
min_sim_length = length(t_sim);

for c = 1:numel(controllers_to_compare)
    res = controllers_to_compare(c).results;
    q_data = res.simulation.q;
    
    % Determine length assuming data might be (Time x DOF) or (DOF x Time)
    if size(q_data, 2) == 7 % Time x DOF
        q_len = size(q_data, 1);
    else % Likely DOF x Time
        q_len = size(q_data, 2);
    end
    min_sim_length = min(min_sim_length, q_len);
end

num_points = min_sim_length;
t_sim = t_sim(1:num_points); % Truncate time vector
gripper_state = [0.02, 0.02]; % Fixed gripper state for visualization

fprintf('Synchronized simulation length: %d time steps.\n', num_points);
% ------------------------------------------------------


%% augment data and pre-calculate EE positions
ee_name = 'tcp';
fprintf('Setting up robot model and calculating EE positions...\n');
try
    robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
catch ME
    error('ERROR: You must have the Robotics System Toolbox installed.');
end
tcp_body = rigidBody('tcp');
tcp_offset = [0, 0, 0.107]; 
setFixedTransform(tcp_body.Joint, trvec2tform(tcp_offset));
addBody(robot, tcp_body, 'panda_hand');


for c = 1:numel(controllers_to_compare)
    res = controllers_to_compare(c).results;
    
    q_sim_7dof  = res.simulation.q;
    q_desired_7dof = res.desired.q;
    
    % Standardize orientation to (Time x DOF) if needed
    if size(q_sim_7dof, 2) ~= 7
         q_sim_7dof  = res.simulation.q';
         q_desired_7dof = res.desired.q';
    end
    
    % Truncate data to the synchronized length
    q_sim_7dof = q_sim_7dof(1:num_points, :);
    q_desired_7dof = q_desired_7dof(1:num_points, :);

    % Concatenate 7 DOF joint positions with fixed 2 DOF gripper state
    controllers_to_compare(c).q_sim     = [q_sim_7dof, repmat(gripper_state, num_points, 1)];
    controllers_to_compare(c).q_desired = [q_desired_7dof, repmat(gripper_state, num_points, 1)];
    
    % Pre-calculate EE positions
    pos_desired_ee = zeros(num_points, 3);
    pos_actual_ee  = zeros(num_points, 3);
    
    fprintf('Computing end-effector positions for %s...\n', controllers_to_compare(c).name);
    for i = 1:num_points
        pos_desired_ee(i,:) = tform2trvec(getTransform(robot, controllers_to_compare(c).q_desired(i,:), ee_name));
        pos_actual_ee(i,:)  = tform2trvec(getTransform(robot, controllers_to_compare(c).q_sim(i,:), ee_name));
    end
    
    controllers_to_compare(c).pos_desired_ee = pos_desired_ee;
    controllers_to_compare(c).pos_actual_ee = pos_actual_ee;
end


%% figure setup
fig = figure('Name', 'Franka trajectory comparison', ...
    'Position', [50 50 1600 800], 'Color', 'w');
sgtitle(sprintf('Trajectory comparison: %s vs %s', CONTROLLER_A, CONTROLLER_B), 'FontSize', 16, 'FontWeight', 'bold');

% Subplot A (Controller A)
ax_A = subplot(1, 2, 1);
show(robot, controllers_to_compare(1).q_sim(1,:), 'Parent', ax_A, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold(ax_A, 'on'); grid(ax_A, 'on'); axis(ax_A, 'equal');
title(ax_A, sprintf('Controller: %s', CONTROLLER_A), 'FontSize', 12);
xlabel(ax_A, 'X [m]'); ylabel(ax_A, 'Y [m]'); zlabel(ax_A, 'Z [m]');
view(ax_A, 135, 25);

% Subplot B (Controller B)
ax_B = subplot(1, 2, 2);
show(robot, controllers_to_compare(2).q_sim(1,:), 'Parent', ax_B, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold(ax_B, 'on'); grid(ax_B, 'on'); axis(ax_B, 'equal');
title(ax_B, sprintf('Controller: %s', CONTROLLER_B), 'FontSize', 12);
xlabel(ax_B, 'X [m]'); ylabel(ax_B, 'Y [m]'); zlabel(ax_B, 'Z [m]');
view(ax_B, 135, 25);

% Link camera views and set limits
linkprop([ax_A, ax_B], {'CameraPosition','CameraUpVector','CameraTarget','XLim','YLim','ZLim'});
xlim(ax_A, [-0.8, 0.8]); ylim(ax_A, [-0.8, 0.8]); zlim(ax_A, [-0.2, 1.3]);


%% Plot initial trajectories and markers
axes_list = {ax_A, ax_B};
plot_handles = cell(2, 1);

for c = 1:numel(controllers_to_compare)
    ax = axes_list{c};
    pos_desired_ee = controllers_to_compare(c).pos_desired_ee;
    pos_actual_ee = controllers_to_compare(c).pos_actual_ee;
    
    h = struct();
    h.desired = plot3(ax, pos_desired_ee(:,1), pos_desired_ee(:,2), pos_desired_ee(:,3), '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 2.5);
    h.actual_trail = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.1 0.4 0.8, 0.8], 'LineWidth', 2);
    h.ee_marker = plot3(ax, pos_actual_ee(1,1), pos_actual_ee(1,2), pos_actual_ee(1,3), 'o', 'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
    plot_handles{c} = h;
end


%% animation
fprintf('Starting animation...\n');

animation_fps = 60;
speedup_factor = 1;
simulation_dt = mean(diff(t_sim));
frame_skip = max(1, round((1/animation_fps) / simulation_dt));
animation_timer = tic;

for i = 1:frame_skip:num_points
    current_sim_time = t_sim(i);

    for c = 1:numel(controllers_to_compare)
        ax = axes_list{c};
        q_current = controllers_to_compare(c).q_sim(i,:);
        pos_current_ee = controllers_to_compare(c).pos_actual_ee(i,:);
        
        % Update robot visual 
        show(robot, q_current, 'Parent', ax, 'PreservePlot', false, 'Frames','off','Visuals','on');
        
        % Update trajectory trail and marker
        set(plot_handles{c}.actual_trail, 'XData', controllers_to_compare(c).pos_actual_ee(1:i,1), ...
                                          'YData', controllers_to_compare(c).pos_actual_ee(1:i,2), ...
                                          'ZData', controllers_to_compare(c).pos_actual_ee(1:i,3));
        set(plot_handles{c}.ee_marker, 'XData', pos_current_ee(1), ...
                                       'YData', pos_current_ee(2), ...
                                       'ZData', pos_current_ee(3));
    end

    sgtitle(sprintf('Trajectory comparison: %s vs %s | Time: %.2f s / %.2f s', ...
        CONTROLLER_A, CONTROLLER_B, current_sim_time, t_sim(end)));
    drawnow;
    
    % Time control for animation speed
    elapsed_real_time = toc(animation_timer);
    time_to_wait = (current_sim_time/speedup_factor) - elapsed_real_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end

sgtitle(sprintf('Animation complete (Final time: %.2f s)', t_sim(end)), 'Color', [0 .6 0]);
fprintf('\nAnimation complete.\n');