clear; clc; close all;

fprintf('=== GENERATING FBL vs PBC DIFFERENCE PLOTS ===\n\n');

%% Setup paths
try
    this_script_path = mfilename('fullpath');
    [script_dir, ~, ~] = fileparts(this_script_path);
    [project_root, ~, ~] = fileparts(script_dir);
catch
    project_root = pwd;
end

results_dir = fullfile(project_root, 'results');
plots_base_dir = fullfile(project_root, 'plots');
diff_dir = fullfile(plots_base_dir, 'Difference');

fprintf('Project root: %s\n', project_root);
fprintf('Results dir: %s\n', results_dir);
fprintf('Plots dir: %s\n', plots_base_dir);
fprintf('Difference dir: %s\n\n', diff_dir);

% Create base difference directory
if ~exist(diff_dir, 'dir')
    mkdir(diff_dir);
end

%% Define structure to iterate
uncertainty_levels = {'high', 'extreme'};
conditions = {'matched', 'mismatched'};
optimizations = {'optimized', 'unoptimized'};
trajectories = {'quintic', 'bang_bang', 'bang_coast_bang'};
trajectory_prefixes = {'quintic', 'bb', 'bcb'}; 
labels = {'A', 'B'};

joint_names = {'Base (J1)', 'Shoulder (J2)', 'Elbow (J3)', 'Forearm (J4)', ...
               'Wrist1 (J5)', 'Wrist2 (J6)', 'Wrist3 (J7)'};

total_plots = 0;
generated_plots = 0;

%% Iterate through all combinations
for u_idx = 1:length(uncertainty_levels)
    unc = uncertainty_levels{u_idx};
    fprintf('Processing uncertainty level: %s\n', unc);
    
    for c_idx = 1:length(conditions)
        cond = conditions{c_idx};
        
        for o_idx = 1:length(optimizations)
            opt = optimizations{o_idx};
            
            for t_idx = 1:length(trajectories)
                traj = trajectories{t_idx};
                traj_prefix = trajectory_prefixes{t_idx};
                
                for l_idx = 1:length(labels)
                    lbl = labels{l_idx};
                    
                    % Construct file paths using the short prefixes
                    fbl_result_path = fullfile(results_dir, 'FBL', unc, cond, opt, traj, sprintf('FBL_%s_%s.mat', traj_prefix, lbl));
                    pbc_result_path = fullfile(results_dir, 'PBC', unc, cond, opt, traj, sprintf('PBC_%s_%s.mat', traj_prefix, lbl));
                    
                    % Debug: Print first path check
                    if u_idx == 1 && c_idx == 1 && o_idx == 1 && t_idx == 1 && l_idx == 1
                        fprintf('Checking first file pair:\n');
                        fprintf('  FBL: %s (exists: %d)\n', fbl_result_path, exist(fbl_result_path, 'file'));
                        fprintf('  PBC: %s (exists: %d)\n', pbc_result_path, exist(pbc_result_path, 'file'));
                    end
                    
                    % Check if both files exist
                    if ~exist(fbl_result_path, 'file') || ~exist(pbc_result_path, 'file')
                        continue;
                    end
                    
                    total_plots = total_plots + 1;
                    
                    % Load results
                    try
                        fbl_data = load(fbl_result_path);
                        pbc_data = load(pbc_result_path);
                        
                        if ~isfield(fbl_data, 'results') || ~isfield(pbc_data, 'results')
                            warning('Missing results field in %s or %s', fbl_result_path, pbc_result_path);
                            continue;
                        end
                        
                        fbl = fbl_data.results;
                        pbc = pbc_data.results;
                        
                        % Create output directory
                        output_dir = fullfile(diff_dir, unc, cond, opt, traj);
                        if ~exist(output_dir, 'dir')
                            mkdir(output_dir);
                        end
                        
                        fprintf('  Generating difference plots: %s/%s/%s/%s-%s\n', unc, cond, opt, traj, lbl);
                        
                        %% Interpolate to common time grid if needed
                        t_fbl = fbl.simulation.t;
                        t_pbc = pbc.simulation.t;
                        
                        % Use the common time range
                        t_start = max(t_fbl(1), t_pbc(1));
                        t_end = min(t_fbl(end), t_pbc(end));
                        
                        % Create common time vector with reasonable resolution
                        dt = min(mean(diff(t_fbl)), mean(diff(t_pbc)));
                        t_common = t_start:dt:t_end;
                        
                        % Interpolate FBL data
                        q_fbl_interp = zeros(7, length(t_common));
                        qd_fbl_interp = zeros(7, length(t_common));
                        ee_fbl_interp = zeros(3, length(t_common));
                        ee_err_fbl_interp = interp1(t_fbl, fbl.errors.ee_mm, t_common, 'linear');
                        
                        for j = 1:7
                            q_fbl_interp(j,:) = interp1(t_fbl, fbl.simulation.q(j,:), t_common, 'linear');
                            qd_fbl_interp(j,:) = interp1(t_fbl, fbl.simulation.qd(j,:), t_common, 'linear');
                        end
                        for j = 1:3
                            ee_fbl_interp(j,:) = interp1(t_fbl, fbl.simulation.ee_pos(j,:), t_common, 'linear');
                        end
                        
                        % Interpolate PBC data
                        q_pbc_interp = zeros(7, length(t_common));
                        qd_pbc_interp = zeros(7, length(t_common));
                        ee_pbc_interp = zeros(3, length(t_common));
                        ee_err_pbc_interp = interp1(t_pbc, pbc.errors.ee_mm, t_common, 'linear');
                        
                        for j = 1:7
                            q_pbc_interp(j,:) = interp1(t_pbc, pbc.simulation.q(j,:), t_common, 'linear');
                            qd_pbc_interp(j,:) = interp1(t_pbc, pbc.simulation.qd(j,:), t_common, 'linear');
                        end
                        for j = 1:3
                            ee_pbc_interp(j,:) = interp1(t_pbc, pbc.simulation.ee_pos(j,:), t_common, 'linear');
                        end
                        
                        %% PLOT 1: Joint Position Comparison
                        t_sim = t_common;
                        
                        % Interpolate desired trajectories to common time grid
                        q_des_fbl_interp = interp1(t_fbl, fbl.desired.q', t_common, 'linear')';  % 7 x N
                        
                        % Paper-quality settings
                        width_cm = 8.8;
                        font_size = 10;
                        line_width = 1.2;
                        
                        figure('Units', 'centimeters', 'Position', [5 5 width_cm 16], ...
                               'PaperUnits', 'centimeters', 'PaperPosition', [0 0 width_cm 16], ...
                               'PaperSize', [width_cm 16], 'Visible', 'off');
                               
                        for joint = 1:7
                            subplot(7, 1, joint);
                            hold on;
                            plot(t_sim, q_des_fbl_interp(joint,:), 'g--', 'LineWidth', line_width);
                            plot(t_sim, q_fbl_interp(joint,:), 'r-', 'LineWidth', line_width);
                            plot(t_sim, q_pbc_interp(joint,:), 'b-', 'LineWidth', line_width);
                            
                            grid on; box on;
                            set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman', 'LineWidth', 0.8);
                            
                            ylabel('rad', 'FontSize', font_size);
                            if joint == 1
                                title('Joint Position Comparison', 'FontSize', font_size+1);
                                lgd = legend({'Ref', 'FBL', 'PBC'}, 'Location', 'best', 'FontSize', font_size-2);
                                lgd.ItemTokenSize = [10, 10];
                            end
                            if joint == 7
                                xlabel('Time [s]', 'FontSize', font_size);
                            end
                            xlim([t_sim(1) t_sim(end)]);
                        end
                        exportgraphics(gcf, fullfile(output_dir, sprintf('joint_comparison_%s.png', lbl)), 'Resolution', 300);
                        close(gcf);
                        
                        %% PLOT 2: End-Effector Position Comparison
                        ee_fbl_mm = ee_fbl_interp * 1000;  % 3 x N, in mm
                        ee_pbc_mm = ee_pbc_interp * 1000;  % 3 x N, in mm
                        
                        % Interpolate desired EE trajectory to common time grid
                        ee_des_interp = interp1(t_fbl, fbl.desired.ee_pos', t_common, 'linear')';  % 3 x N
                        ee_des_mm = ee_des_interp * 1000;  % in mm
                        
                        figure('Units', 'centimeters', 'Position', [5 5 12 9], ...
                               'PaperUnits', 'centimeters', 'PaperPosition', [0 0 12 9], ...
                               'PaperSize', [12 9], 'Visible', 'off');
                        
                        % 3D trajectories comparison
                        subplot(2, 2, 1);
                        hold on;
                        plot3(ee_des_mm(1,:), ee_des_mm(2,:), ee_des_mm(3,:), 'g--', 'LineWidth', line_width);
                        plot3(ee_fbl_mm(1,:), ee_fbl_mm(2,:), ee_fbl_mm(3,:), 'r-', 'LineWidth', line_width);
                        plot3(ee_pbc_mm(1,:), ee_pbc_mm(2,:), ee_pbc_mm(3,:), 'b-', 'LineWidth', line_width);
                        plot3(ee_fbl_mm(1,1), ee_fbl_mm(2,1), ee_fbl_mm(3,1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
                        plot3(ee_fbl_mm(1,end), ee_fbl_mm(2,end), ee_fbl_mm(3,end), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
                        
                        grid on; box on; axis equal; view(3);
                        set(gca, 'FontSize', font_size-1, 'FontName', 'Times New Roman');
                        title('3D Path', 'FontSize', font_size);
                        xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
                        
                        % X, Y, Z components comparison
                        subplot(2, 2, 2);
                        hold on;
                        plot(t_sim, ee_des_mm(1,:), 'g--', 'LineWidth', line_width);
                        plot(t_sim, ee_fbl_mm(1,:), 'r-', 'LineWidth', line_width);
                        plot(t_sim, ee_pbc_mm(1,:), 'b-', 'LineWidth', line_width);
                        grid on; box on;
                        set(gca, 'FontSize', font_size-1, 'FontName', 'Times New Roman');
                        title('X Pos', 'FontSize', font_size);
                        xlabel('Time [s]'); ylabel('X [mm]');
                        
                        subplot(2, 2, 3);
                        hold on;
                        plot(t_sim, ee_des_mm(2,:), 'g--', 'LineWidth', line_width);
                        plot(t_sim, ee_fbl_mm(2,:), 'r-', 'LineWidth', line_width);
                        plot(t_sim, ee_pbc_mm(2,:), 'b-', 'LineWidth', line_width);
                        grid on; box on;
                        set(gca, 'FontSize', font_size-1, 'FontName', 'Times New Roman');
                        title('Y Pos', 'FontSize', font_size);
                        xlabel('Time [s]'); ylabel('Y [mm]');
                        
                        subplot(2, 2, 4);
                        hold on;
                        plot(t_sim, ee_des_mm(3,:), 'g--', 'LineWidth', line_width);
                        plot(t_sim, ee_fbl_mm(3,:), 'r-', 'LineWidth', line_width);
                        plot(t_sim, ee_pbc_mm(3,:), 'b-', 'LineWidth', line_width);
                        grid on; box on;
                        set(gca, 'FontSize', font_size-1, 'FontName', 'Times New Roman');
                        title('Z Pos', 'FontSize', font_size);
                        xlabel('Time [s]'); ylabel('Z [mm]');
                        
                        exportgraphics(gcf, fullfile(output_dir, sprintf('ee_comparison_%s.png', lbl)), 'Resolution', 300);
                        close(gcf);
                        
                        %% PLOT 3: Tracking Error Comparison
                        % Interpolate error norms to common time grid
                        fbl_pos_error_norm = sqrt(sum(fbl.errors.pos.^2, 1));
                        pbc_pos_error_norm = sqrt(sum(pbc.errors.pos.^2, 1));
                        fbl_pos_error_interp = interp1(t_fbl, fbl_pos_error_norm, t_common, 'linear');
                        pbc_pos_error_interp = interp1(t_pbc, pbc_pos_error_norm, t_common, 'linear');
                        
                        fbl_vel_error_norm = sqrt(sum(fbl.errors.vel.^2, 1));
                        pbc_vel_error_norm = sqrt(sum(pbc.errors.vel.^2, 1));
                        fbl_vel_error_interp = interp1(t_fbl, fbl_vel_error_norm, t_common, 'linear');
                        pbc_vel_error_interp = interp1(t_pbc, pbc_vel_error_norm, t_common, 'linear');
                        
                        height_cm = 6;
                        figure('Units', 'centimeters', 'Position', [5 5 width_cm height_cm], ...
                               'PaperUnits', 'centimeters', 'PaperPosition', [0 0 width_cm height_cm], ...
                               'PaperSize', [width_cm height_cm], 'Visible', 'off');
                        
                        hold on;
                        % Plot position error on left axis
                        yyaxis left
                        plot(t_sim, fbl_pos_error_interp * 1000, 'r-', 'LineWidth', line_width);
                        plot(t_sim, pbc_pos_error_interp * 1000, 'r--', 'LineWidth', line_width);
                        ylabel('Pos Error ||e|| [mrad]', 'FontSize', font_size);
                        set(gca, 'YColor', 'r');
                        
                        % Plot velocity error on right axis
                        yyaxis right
                        plot(t_sim, fbl_vel_error_interp * 1000, 'b-', 'LineWidth', line_width);
                        plot(t_sim, pbc_vel_error_interp * 1000, 'b--', 'LineWidth', line_width);
                        ylabel('Vel Error ||e_v|| [mrad/s]', 'FontSize', font_size);
                        set(gca, 'YColor', 'b');
                        
                        grid on; box on;
                        set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman', 'LineWidth', 0.8);
                        xlabel('Time [s]', 'FontSize', font_size);
                        title('Tracking Errors Comparison', 'FontSize', font_size+1);
                        lgd = legend({'FBL Pos', 'PBC Pos', 'FBL Vel', 'PBC Vel'}, 'Location', 'best', 'FontSize', font_size-2);
                        lgd.ItemTokenSize = [10, 10];
                        
                        exportgraphics(gcf, fullfile(output_dir, sprintf('error_comparison_%s.png', lbl)), 'Resolution', 300);
                        close(gcf);
                        
                        %% PLOT 4: Control Torque Comparison
                        % Check if torque data exists (for backwards compatibility)
                        if isfield(fbl.simulation, 'tau') && isfield(pbc.simulation, 'tau')
                            % Interpolate torques to common time grid
                            tau_fbl_interp = interp1(t_fbl, fbl.simulation.tau', t_common, 'linear')';  % 7 x N
                            tau_pbc_interp = interp1(t_pbc, pbc.simulation.tau', t_common, 'linear')';  % 7 x N
                            
                            figure('Units', 'centimeters', 'Position', [5 5 width_cm 16], ...
                                   'PaperUnits', 'centimeters', 'PaperPosition', [0 0 width_cm 16], ...
                                   'PaperSize', [width_cm 16], 'Visible', 'off');
                            
                            for joint = 1:7
                                subplot(7, 1, joint);
                                hold on;
                                plot(t_common, tau_fbl_interp(joint,:), 'r-', 'LineWidth', line_width);
                                plot(t_common, tau_pbc_interp(joint,:), 'b-', 'LineWidth', line_width);
                                
                                grid on; box on;
                                set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman', 'LineWidth', 0.8);
                                
                                ylabel('Nm', 'FontSize', font_size);
                                if joint == 1
                                    title('Control Torques Comparison', 'FontSize', font_size+1);
                                    lgd = legend({'FBL', 'PBC'}, 'Location', 'best', 'FontSize', font_size-2);
                                    lgd.ItemTokenSize = [10, 10];
                                end
                                if joint == 7
                                    xlabel('Time [s]', 'FontSize', font_size);
                                end
                                xlim([t_common(1) t_common(end)]);
                            end
                            exportgraphics(gcf, fullfile(output_dir, sprintf('torque_comparison_%s.png', lbl)), 'Resolution', 300);
                            close(gcf);
                        else
                            fprintf('  Skipping torque comparison (tau data not available)\n');
                        end
                        
                        %% PLOT 5: End-Effector Error Comparison
                        figure('Units', 'centimeters', 'Position', [5 5 12 6], ...
                               'PaperUnits', 'centimeters', 'PaperPosition', [0 0 12 6], ...
                               'PaperSize', [12 6], 'Visible', 'off');
                        
                        subplot(1, 2, 1);
                        hold on;
                        plot(t_sim, ee_err_fbl_interp, 'r-', 'LineWidth', line_width);
                        plot(t_sim, ee_err_pbc_interp, 'b-', 'LineWidth', line_width);
                        grid on; box on;
                        set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman', 'LineWidth', 0.8);
                        title('EE Error Over Time', 'FontSize', font_size);
                        xlabel('Time [s]', 'FontSize', font_size);
                        ylabel('EE Error [mm]', 'FontSize', font_size);
                        lgd = legend({'FBL', 'PBC'}, 'Location', 'best', 'FontSize', font_size-2);
                        lgd.ItemTokenSize = [10, 10];
                        
                        subplot(1, 2, 2);
                        % Bar chart for better comparison
                        categories = {'Max', 'RMS', 'Final'};
                        fbl_values = [fbl.performance.ee_max_error_mm, fbl.performance.ee_rms_error_mm, fbl.performance.ee_final_error_mm];
                        pbc_values = [pbc.performance.ee_max_error_mm, pbc.performance.ee_rms_error_mm, pbc.performance.ee_final_error_mm];
                        
                        x = 1:3;
                        bar(x-0.15, fbl_values, 0.3, 'FaceColor', 'r');
                        hold on;
                        bar(x+0.15, pbc_values, 0.3, 'FaceColor', 'b');
                        set(gca, 'XTick', x, 'XTickLabel', categories);
                        set(gca, 'FontSize', font_size, 'FontName', 'Times New Roman', 'LineWidth', 0.8);
                        ylabel('Error [mm]', 'FontSize', font_size);
                        title('Performance Metrics', 'FontSize', font_size);
                        lgd = legend({'FBL', 'PBC'}, 'Location', 'best', 'FontSize', font_size-2);
                        lgd.ItemTokenSize = [10, 10];
                        grid on; box on;
                        
                        exportgraphics(gcf, fullfile(output_dir, sprintf('ee_metrics_%s.png', lbl)), 'Resolution', 300);
                        close(gcf);
                        
                        generated_plots = generated_plots + 1;
                        
                    catch ME
                        warning('Error processing %s: %s', fbl_result_path, ME.message);
                    end
                end
            end
        end
    end
end

fprintf('\n=== SUMMARY ===\n');
fprintf('Total comparisons attempted: %d\n', total_plots);
fprintf('Successfully generated: %d\n', generated_plots);
fprintf('Difference plots saved to: %s\n', diff_dir);
