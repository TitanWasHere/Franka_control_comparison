clear; clc; close all;

fprintf('=== ANIMAZIONE TRAIETTORIA (VISTA DOPPIA) ===\n\n');

%% config params
CONTROLLER = "FBL";
TRAJ = 'B';
MATCHED_START = true;
OPTIMIZED_GAINS = false;
PROFILE = "bang_bang"; % "quintic", "bang_bang", "bang_coast_bang"

%% path setup
try
    this_script_path = mfilename('fullpath');
    [script_dir, ~, ~] = fileparts(this_script_path);
    [project_root, ~, ~] = fileparts(script_dir);
catch
    project_root = pwd; 
    warning('Could not determine project root automatically. Assuming current directory is project root.');
end

sub_path = sprintf("%s/", CONTROLLER); 
if MATCHED_START, sub_path = strcat(sub_path, "matched/"); else, sub_path = strcat(sub_path, "mismatched/"); end
if OPTIMIZED_GAINS, sub_path = strcat(sub_path, "optimized/"); else, sub_path = strcat(sub_path, "unoptimized/"); end
sub_path = strcat(sub_path, sprintf("%s/", PROFILE));

switch PROFILE
    case "quintic", traj_suffix = "quintic";
    case "bang_bang", traj_suffix = "bb";
    case "bang_coast_bang", traj_suffix = "bcb";
    otherwise, error('Unknown PROFILE type.');
end
file_name = sprintf("%s_%s_%s.mat", CONTROLLER, traj_suffix, TRAJ);
path = fullfile(project_root, 'results', sub_path, file_name);


%% data loading
fprintf('Loading results file ''%s''...\n', path);
try
    load(path, 'results');
    fprintf('File loaded succesfully.\n');
catch ME
    fprintf('Could not find the results file.\n');
    rethrow(ME);
end

t_sim       = results.simulation.t;
q_sim_7dof  = results.simulation.q';
q_desired_7dof = results.desired.q';

%% augment config
gripper_state = [0.02, 0.02];
num_points = size(q_sim_7dof, 1);
q_sim     = [q_sim_7dof, repmat(gripper_state, num_points, 1)];
q_desired = [q_desired_7dof, repmat(gripper_state, num_points, 1)];

%% robot model and figure setup
fprintf('Setting up robot model and visualization environment...\n');
try
    robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
catch ME
    fprintf('ERROR: you have the Robotics System Toolbox installed.\n');
    rethrow(ME);
end

tcp_body = rigidBody('tcp');
tcp_offset = [0, 0, 0.107]; 
setFixedTransform(tcp_body.Joint, trvec2tform(tcp_offset));
addBody(robot, tcp_body, 'panda_hand');

fig = figure('Name', 'Franka trajectory animation', ...
    'Position', [50 50 1600 800], 'Color', 'w');
sgtitle(sprintf('Trajectory analysis %s', CONTROLLER), 'FontSize', 16, 'FontWeight', 'bold');

ax1 = subplot(1, 2, 1);
show(robot, q_sim(1,:), 'Parent', ax1, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Realistic view', 'FontSize', 12);
xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]'); zlabel(ax1, 'Z [m]');
view(ax1, 135, 25);

ax2 = subplot(1, 2, 2);
show(robot, q_sim(1,:), 'Parent', ax2, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'off');
hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
title(ax2, 'Kinematic view', 'FontSize', 12);
xlabel(ax2, 'X [m]'); ylabel(ax2, 'Y [m]'); zlabel(ax2, 'Z [m]');
view(ax2, 135, 25);

bodies = setdiff(robot.BodyNames, {'panda_leftfinger', 'panda_rightfinger', 'panda_gripper'});
n_bodies = numel(bodies);
joint_markers = gobjects(n_bodies,1);
link_lines    = gobjects(n_bodies,1);
for b = 1:n_bodies
    body_name = bodies{b};
    if strcmp(body_name, 'base'), continue; end
    T = getTransform(robot, q_sim(1,:), body_name);
    p = tform2trvec(T);
    joint_markers(b) = plot3(ax2, p(1), p(2), p(3), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
    parentBody = robot.getBody(body_name).Parent;
    if ~isempty(parentBody)
        Tp = getTransform(robot, q_sim(1,:), parentBody.Name);
        pp = tform2trvec(Tp);
        link_lines(b) = plot3(ax2, [pp(1) p(1)], [pp(2) p(2)], [pp(3) p(3)], 'k-', 'LineWidth', 2);
    end
end
linkprop([ax1, ax2], {'CameraPosition','CameraUpVector','CameraTarget','XLim','YLim','ZLim'});
xlim(ax1, [-0.8, 0.8]); ylim(ax1, [-0.8, 0.8]); zlim(ax1, [-0.2, 1.3]);

%% end-effector positions
fprintf('Computing end-effector positions...\n');
ee_name = 'tcp';
pos_desired_ee = zeros(num_points, 3);
pos_actual_ee  = zeros(num_points, 3);
for i = 1:num_points
    % Calcola la trasformazione per ogni punto temporale, una riga alla volta
    pos_desired_ee(i,:) = tform2trvec(getTransform(robot, q_desired(i,:), ee_name));
    pos_actual_ee(i,:)  = tform2trvec(getTransform(robot, q_sim(i,:), ee_name));
end

%% trajectories
axes_list = [ax1, ax2];
plot_handles = struct();
for i = 1:length(axes_list)
    ax = axes_list(i);
    plot_handles(i).desired = plot3(ax, pos_desired_ee(:,1), pos_desired_ee(:,2), pos_desired_ee(:,3), '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 2.5);
    plot_handles(i).actual_trail = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.1 0.4 0.8, 0.8], 'LineWidth', 2);
    plot_handles(i).ee_marker = plot3(ax, pos_actual_ee(1,1), pos_actual_ee(1,2), pos_actual_ee(1,3), 'o', 'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
end

%% animation
fprintf('Starting animation...\n');

animation_fps = 30;
speedup_factor = 1.5;
simulation_dt = mean(diff(t_sim));
frame_skip = max(1, round((1/animation_fps) / simulation_dt));
animation_timer = tic;

for i = 1:frame_skip:num_points
    current_sim_time = t_sim(i);
    q_current = q_sim(i,:);

    show(robot, q_current, 'Parent', ax1, 'PreservePlot', false, 'Frames','off','Visuals','on');
    
    for ax_idx = 1:2
        set(plot_handles(ax_idx).actual_trail, 'XData', pos_actual_ee(1:i,1), 'YData', pos_actual_ee(1:i,2), 'ZData', pos_actual_ee(1:i,3));
        set(plot_handles(ax_idx).ee_marker, 'XData', pos_actual_ee(i,1), 'YData', pos_actual_ee(i,2), 'ZData', pos_actual_ee(i,3));
    end

    for b = 1:n_bodies
        body_name = bodies{b};
        if strcmp(body_name, 'base'), continue; end
        T = getTransform(robot, q_current, body_name);
        p = tform2trvec(T);
        set(joint_markers(b), 'XData', p(1), 'YData', p(2), 'ZData', p(3));
        parentBody = robot.getBody(body_name).Parent;
        if ~isempty(parentBody)
            Tp = getTransform(robot, q_current, parentBody.Name);
            pp = tform2trvec(Tp);
            set(link_lines(b), 'XData', [pp(1) p(1)], 'YData', [pp(2) p(2)], 'ZData', [pp(3) p(3)]);
        end
    end

    sgtitle(sprintf('Trajectory analysis %s - Tempo: %.2f s / %.2f s', CONTROLLER, current_sim_time, t_sim(end)));
    drawnow;
    
    elapsed_real_time = toc(animation_timer);
    time_to_wait = (current_sim_time/speedup_factor) - elapsed_real_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end

sgtitle(sprintf('Animation complete (Final time: %.2f s)', t_sim(end)), 'Color', [0 .6 0]);
fprintf('\nAnimation complete.\n');