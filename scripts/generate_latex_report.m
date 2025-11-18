clear; clc; close all;

fprintf('=== GENERATING LATEX COMPARISON REPORT ===\n\n');

%% Setup paths
try
    this_script_path = mfilename('fullpath');
    [script_dir, ~, ~] = fileparts(this_script_path);
    [project_root, ~, ~] = fileparts(script_dir);
catch
    project_root = pwd;
end

results_dir = fullfile(project_root, 'results');
output_file = fullfile(project_root, 'comparison_report.tex');

%% Scan all .mat files in results directory
fprintf('Scanning results directory: %s\n', results_dir);
all_mat_files = dir(fullfile(results_dir, '**', '*.mat'));
fprintf('Found %d result files.\n\n', length(all_mat_files));

%% Trajectory and Config Mappings for Readability
traj_map = containers.Map({'quintic', 'bang_bang', 'bang_coast_bang'}, {'Quintic', 'BB', 'BCB'});
config_map = containers.Map({'matched_optimized', 'matched_unoptimized', 'mismatched_optimized', 'mismatched_unoptimized'}, ...
                            {'Match/Opt', 'Match/Unopt', 'Mismatch/Opt', 'Mismatch/Unopt'});
config_map_short = containers.Map({'matched_optimized', 'matched_unoptimized', 'mismatched_optimized', 'mismatched_unoptimized'}, ...
                                  {'Match-Opt', 'Match-Unopt', 'Mismatch-Opt', 'Mismatch-Unopt'});

%% Load and organize all results
results_database = struct();
categories = {'controller', 'uncertainty', 'matched', 'optimized', 'trajectory', 'label'};

for i = 1:length(all_mat_files)
    file_path = fullfile(all_mat_files(i).folder, all_mat_files(i).name);
    
    % Parse path to extract metadata
    rel_path = strrep(all_mat_files(i).folder, results_dir, '');
    path_parts = strsplit(rel_path, filesep);
    path_parts = path_parts(~cellfun('isempty', path_parts));
    
    if length(path_parts) < 5
        warning('Skipping file with unexpected path structure: %s', file_path);
        continue;
    end
    
    % Extract metadata from path
    metadata.controller = path_parts{1};      % FBL or PBC
    metadata.uncertainty = path_parts{2};     % friction, low, mid, high, extreme
    metadata.matched = path_parts{3};         % matched or mismatched
    metadata.optimized = path_parts{4};       % optimized or unoptimized
    metadata.trajectory = path_parts{5};      % quintic, bang_bang, bang_coast_bang
    
    % Extract label from filename (A or B)
    [~, fname, ~] = fileparts(all_mat_files(i).name);
    metadata.label = fname(end);
    
    % Load results
    try
        loaded = load(file_path);
        if ~isfield(loaded, 'results')
            warning('File %s does not contain results variable.', file_path);
            continue;
        end
        
        % Extract key metrics
        metrics.joint_max_error_rad = loaded.results.performance.joint_max_error_rad;
        metrics.joint_rms_error_rad = loaded.results.performance.joint_rms_error_rad;
        metrics.ee_max_error_mm = loaded.results.performance.ee_max_error_mm;
        metrics.ee_rms_error_mm = loaded.results.performance.ee_rms_error_mm;
        metrics.ee_final_error_mm = loaded.results.performance.ee_final_error_mm;
        metrics.simulation_time = loaded.results.simulation_time;
        
        % Additional metrics (using a placeholder for tau for structure consistency)
        tau_computed = zeros(length(loaded.results.simulation.t), 7);
        for j = 1:length(loaded.results.simulation.t)
            % Compute torque norm
            tau_computed(j,:) = loaded.results.errors.pos(:,j)'; % placeholder - would need actual tau
        end
        metrics.max_joint_error_deg = rad2deg(metrics.joint_max_error_rad);
        metrics.rms_joint_error_deg = rad2deg(metrics.joint_rms_error_rad);
        
        % Store in database
        key = sprintf('%s_%s_%s_%s_%s_%s', metadata.controller, metadata.uncertainty, ...
                      metadata.matched, metadata.optimized, metadata.trajectory, metadata.label);
        results_database.(key).metadata = metadata;
        results_database.(key).metrics = metrics;
        results_database.(key).file_path = file_path;
        
        fprintf('  Loaded: %s\n', key);
        
    catch ME
        warning('Error loading file %s: %s', file_path, ME.message);
    end
end

%% Generate LaTeX report
fprintf('\nGenerating LaTeX report...\n');
fid = fopen(output_file, 'w');

% LaTeX preamble
fprintf(fid, '\\documentclass[11pt,a4paper]{article}\n');
fprintf(fid, '\\usepackage[utf8]{inputenc}\n');
fprintf(fid, '\\usepackage{geometry}\n');
fprintf(fid, '\\geometry{margin=2cm}\n');
fprintf(fid, '\\usepackage{booktabs}\n');
fprintf(fid, '\\usepackage{array}\n');
fprintf(fid, '\\usepackage{xcolor}\n');
fprintf(fid, '\\usepackage{longtable}\n');
fprintf(fid, '\\usepackage{float}\n');
fprintf(fid, '\\usepackage{graphicx}\n');
fprintf(fid, '\\usepackage{amsmath}\n');
fprintf(fid, '\\definecolor{bestcolor}{RGB}{34,139,34}\n'); % Forest Green
fprintf(fid, '\\definecolor{worstcolor}{RGB}{178,34,34}\n'); % Firebrick
fprintf(fid, '\n');
fprintf(fid, '\\title{Franka Emika Robot Control Comparison Report}\n');
fprintf(fid, '\\author{Automatic Analysis}\n');
fprintf(fid, '\\date{\\today}\n');
fprintf(fid, '\n');
fprintf(fid, '\\begin{document}\n');
fprintf(fid, '\\maketitle\n');
fprintf(fid, '\\tableofcontents\n');
fprintf(fid, '\\newpage\n\n');

%% Section 1: Overview
fprintf(fid, '\\section{Overview}\n\n');
fprintf(fid, 'This report presents a comprehensive comparison of robot control performance across different:\n');
fprintf(fid, '\\begin{itemize}\n');
fprintf(fid, '  \\item Controllers: FBL (Feedback Linearization) and PBC (Passivity-Based Control)\n');
fprintf(fid, '  \\item Uncertainty levels: high, extreme\n');
fprintf(fid, '  \\item Initial conditions: matched vs mismatched\n');
fprintf(fid, '  \\item Gain tuning: optimized vs unoptimized\n');
fprintf(fid, '  \\item Trajectory types: quintic, bang-bang, bang-coast-bang\n');
fprintf(fid, '\\end{itemize}\n\n');
fprintf(fid, 'Total simulations analyzed: %d\n\n', length(fieldnames(results_database)));

%% Section 2: Summary Statistics by Controller
controllers = {'FBL', 'PBC'};
fprintf(fid, '\\section{Controller Performance Summary}\n\n');
for c = 1:length(controllers)
    ctrl = controllers{c};
    fprintf(fid, '\\subsection{%s Controller}\n\n', ctrl);
    
    % Filter results for this controller
    all_keys = fieldnames(results_database);
    ctrl_keys = all_keys(startsWith(all_keys, ctrl));
    
    if isempty(ctrl_keys)
        fprintf(fid, 'No results found for %s controller.\n\n', ctrl);
        continue;
    end
    
    % Collect metrics
    ee_max_errors = [];
    ee_rms_errors = [];
    joint_max_errors = [];
    
    for i = 1:length(ctrl_keys)
        ee_max_errors(i) = results_database.(ctrl_keys{i}).metrics.ee_max_error_mm;
        ee_rms_errors(i) = results_database.(ctrl_keys{i}).metrics.ee_rms_error_mm;
        joint_max_errors(i) = results_database.(ctrl_keys{i}).metrics.max_joint_error_deg;
    end
    
    fprintf(fid, '\\begin{table}[H]\n');
    fprintf(fid, '\\centering\n');
    fprintf(fid, '\\begin{tabular}{lc}\n');
    fprintf(fid, '\\toprule\n');
    fprintf(fid, 'Metric & Value \\\\\n');
    fprintf(fid, '\\midrule\n');
    fprintf(fid, 'Number of simulations & %d \\\\\n', length(ctrl_keys));
    fprintf(fid, 'Mean EE max error [mm] & %.2f \\\\\n', mean(ee_max_errors));
    fprintf(fid, 'Std EE max error [mm] & %.2f \\\\\n', std(ee_max_errors));
    fprintf(fid, 'Best EE max error [mm] & %.2f \\\\\n', min(ee_max_errors));
    fprintf(fid, 'Worst EE max error [mm] & %.2f \\\\\n', max(ee_max_errors));
    fprintf(fid, '\\midrule\n');
    fprintf(fid, 'Mean joint max error [deg] & %.2f \\\\\n', mean(joint_max_errors));
    fprintf(fid, 'Best joint max error [deg] & %.2f \\\\\n', min(joint_max_errors));
    fprintf(fid, 'Worst joint max error [deg] & %.2f \\\\\n', max(joint_max_errors));
    fprintf(fid, '\\bottomrule\n');
    fprintf(fid, '\\end{tabular}\n');
    fprintf(fid, '\\caption{Summary statistics for %s controller}\n', ctrl);
    fprintf(fid, '\\end{table}\n\n');
end

%% Section 3: Detailed Comparison Tables (NEW CLEARER FORMAT)
fprintf(fid, '\\section{Detailed Performance Tables}\n\n');
fprintf(fid, 'Performance tables show results grouped by trajectory type, with test labels A and B, including averages for each configuration. Lower values indicate better performance.\n\n');

% Group by uncertainty level
uncertainty_levels = {'high', 'extreme'};
trajectories = {'quintic', 'bang_bang', 'bang_coast_bang'};
traj_display = {'Quintic', 'Bang-Bang', 'Bang-Coast-Bang'};

for u = 1:length(uncertainty_levels)
    unc = uncertainty_levels{u};
    fprintf(fid, '\\subsection{Uncertainty Level: %s}\n\n', upper(unc));
    
    % For each configuration (matched/mismatched, optimized/unoptimized)
    configs = {'matched_optimized', 'matched_unoptimized', 'mismatched_optimized', 'mismatched_unoptimized'};
    config_titles = {'Matched Initial Conditions, Optimized Gains', ...
                     'Matched Initial Conditions, Unoptimized Gains', ...
                     'Mismatched Initial Conditions, Optimized Gains', ...
                     'Mismatched Initial Conditions, Unoptimized Gains'};
    
    for cfg_idx = 1:length(configs)
        cfg = configs{cfg_idx};
        cfg_title = config_titles{cfg_idx};
        
        fprintf(fid, '\\subsubsection{%s}\n\n', cfg_title);
        
        % Create separate tables for FBL and PBC
        for ctrl_idx = 1:2
            if ctrl_idx == 1
                ctrl = 'FBL';
            else
                ctrl = 'PBC';
            end
            
            fprintf(fid, '\\begin{table}[H]\n');
            fprintf(fid, '\\centering\n');
            fprintf(fid, '\\caption{%s Controller - %s uncertainty - %s}\n', ctrl, unc, cfg_title);
            fprintf(fid, '\\begin{tabular}{lcccc}\n');
            fprintf(fid, '\\toprule\n');
            fprintf(fid, 'Trajectory & Test & RMS Error (rad) & RMS Error (deg) & Final EE Error (mm) \\\\\n');
            fprintf(fid, '\\midrule\n');
            
            % Collect data for each trajectory
            for t_idx = 1:length(trajectories)
                traj = trajectories{t_idx};
                traj_name = traj_display{t_idx};
                
                % Get data for labels A and B
                labels = {'A', 'B'};
                traj_data = [];
                
                for lbl_idx = 1:length(labels)
                    lbl = labels{lbl_idx};
                    key = sprintf('%s_%s_%s_%s_%s', ctrl, unc, strrep(cfg, '_', '_'), traj, lbl);
                    
                    if isfield(results_database, key)
                        m = results_database.(key).metrics;
                        
                        % Print trajectory name only on first row
                        if lbl_idx == 1
                            fprintf(fid, '%s & %s & %.4f & %.2f & %.2f \\\\\n', ...
                                    traj_name, lbl, m.joint_rms_error_rad, m.rms_joint_error_deg, m.ee_final_error_mm);
                        else
                            fprintf(fid, '%s & %s & %.4f & %.2f & %.2f \\\\\n', ...
                                    '', lbl, m.joint_rms_error_rad, m.rms_joint_error_deg, m.ee_final_error_mm);
                        end
                        
                        % Store for average calculation
                        traj_data = [traj_data; m.joint_rms_error_rad, m.rms_joint_error_deg, m.ee_final_error_mm];
                    end
                end
                
                % Add midrule after each trajectory pair (except last)
                if t_idx < length(trajectories)
                    fprintf(fid, '\\midrule\n');
                end
            end
            
            % Calculate and print overall average
            fprintf(fid, '\\midrule\n');
            
            % Collect all data for average
            all_data = [];
            for t_idx = 1:length(trajectories)
                traj = trajectories{t_idx};
                for lbl_idx = 1:2
                    lbl = labels{lbl_idx};
                    key = sprintf('%s_%s_%s_%s_%s', ctrl, unc, strrep(cfg, '_', '_'), traj, lbl);
                    if isfield(results_database, key)
                        m = results_database.(key).metrics;
                        all_data = [all_data; m.joint_rms_error_rad, m.rms_joint_error_deg, m.ee_final_error_mm];
                    end
                end
            end
            
            if ~isempty(all_data)
                avg_vals = mean(all_data, 1);
                fprintf(fid, '\\textbf{Average} & & \\textbf{%.4f} & \\textbf{%.2f} & \\textbf{%.2f} \\\\\n', ...
                        avg_vals(1), avg_vals(2), avg_vals(3));
            end
            
            fprintf(fid, '\\bottomrule\n');
            fprintf(fid, '\\end{tabular}\n');
            fprintf(fid, '\\end{table}\n\n');
        end
    end
end

%% Section 4: FBL vs PBC Direct Comparison
fprintf(fid, '\\section{FBL vs PBC Controller Comparison}\n\n');
fprintf(fid, 'This section provides direct head-to-head comparisons between FBL and PBC controllers under identical conditions.\n\n');

% Create comparison tables for each uncertainty level and configuration
for u = 1:length(uncertainty_levels)
    unc = uncertainty_levels{u};
    fprintf(fid, '\\subsection{Uncertainty Level: %s}\n\n', upper(unc));
    
    configs = {'matched_optimized', 'matched_unoptimized', 'mismatched_optimized', 'mismatched_unoptimized'};
    config_titles = {'Matched Initial Conditions, Optimized Gains', ...
                     'Matched Initial Conditions, Unoptimized Gains', ...
                     'Mismatched Initial Conditions, Optimized Gains', ...
                     'Mismatched Initial Conditions, Unoptimized Gains'};
    
    for cfg_idx = 1:length(configs)
        cfg = configs{cfg_idx};
        cfg_title = config_titles{cfg_idx};
        
        fprintf(fid, '\\subsubsection{%s}\n\n', cfg_title);
        
        fprintf(fid, '\\begin{table}[H]\n');
        fprintf(fid, '\\centering\n');
        fprintf(fid, '\\caption{FBL vs PBC Comparison - %s uncertainty - %s}\n', unc, cfg_title);
        fprintf(fid, '\\begin{tabular}{llcccc}\n');
        fprintf(fid, '\\toprule\n');
        fprintf(fid, 'Trajectory & Controller & Test & RMS Error (rad) & RMS Error (deg) & Final EE Error (mm) \\\\\n');
        fprintf(fid, '\\midrule\n');
        
        % For each trajectory
        for t_idx = 1:length(trajectories)
            traj = trajectories{t_idx};
            traj_name = traj_display{t_idx};
            
            labels = {'A', 'B'};
            
            for lbl_idx = 1:length(labels)
                lbl = labels{lbl_idx};
                
                % Get FBL and PBC data
                fbl_key = sprintf('FBL_%s_%s_%s_%s', unc, strrep(cfg, '_', '_'), traj, lbl);
                pbc_key = sprintf('PBC_%s_%s_%s_%s', unc, strrep(cfg, '_', '_'), traj, lbl);
                
                if isfield(results_database, fbl_key) && isfield(results_database, pbc_key)
                    fbl_m = results_database.(fbl_key).metrics;
                    pbc_m = results_database.(pbc_key).metrics;
                    
                    % Color the better values
                    if fbl_m.joint_rms_error_rad < pbc_m.joint_rms_error_rad
                        fbl_rms_rad_str = sprintf('\\textcolor{bestcolor}{%.4f}', fbl_m.joint_rms_error_rad);
                        pbc_rms_rad_str = sprintf('%.4f', pbc_m.joint_rms_error_rad);
                    else
                        fbl_rms_rad_str = sprintf('%.4f', fbl_m.joint_rms_error_rad);
                        pbc_rms_rad_str = sprintf('\\textcolor{bestcolor}{%.4f}', pbc_m.joint_rms_error_rad);
                    end
                    
                    if fbl_m.rms_joint_error_deg < pbc_m.rms_joint_error_deg
                        fbl_rms_deg_str = sprintf('\\textcolor{bestcolor}{%.2f}', fbl_m.rms_joint_error_deg);
                        pbc_rms_deg_str = sprintf('%.2f', pbc_m.rms_joint_error_deg);
                    else
                        fbl_rms_deg_str = sprintf('%.2f', fbl_m.rms_joint_error_deg);
                        pbc_rms_deg_str = sprintf('\\textcolor{bestcolor}{%.2f}', pbc_m.rms_joint_error_deg);
                    end
                    
                    if fbl_m.ee_final_error_mm < pbc_m.ee_final_error_mm
                        fbl_ee_str = sprintf('\\textcolor{bestcolor}{%.2f}', fbl_m.ee_final_error_mm);
                        pbc_ee_str = sprintf('%.2f', pbc_m.ee_final_error_mm);
                    else
                        fbl_ee_str = sprintf('%.2f', fbl_m.ee_final_error_mm);
                        pbc_ee_str = sprintf('\\textcolor{bestcolor}{%.2f}', pbc_m.ee_final_error_mm);
                    end
                    
                    % Print rows
                    if lbl_idx == 1
                        fprintf(fid, '%s & FBL & %s & %s & %s & %s \\\\\n', ...
                                traj_name, lbl, fbl_rms_rad_str, fbl_rms_deg_str, fbl_ee_str);
                        fprintf(fid, ' & PBC & %s & %s & %s & %s \\\\\n', ...
                                lbl, pbc_rms_rad_str, pbc_rms_deg_str, pbc_ee_str);
                    else
                        fprintf(fid, ' & FBL & %s & %s & %s & %s \\\\\n', ...
                                lbl, fbl_rms_rad_str, fbl_rms_deg_str, fbl_ee_str);
                        fprintf(fid, ' & PBC & %s & %s & %s & %s \\\\\n', ...
                                lbl, pbc_rms_rad_str, pbc_rms_deg_str, pbc_ee_str);
                    end
                end
            end
            
            % Add separator between trajectories
            if t_idx < length(trajectories)
                fprintf(fid, '\\midrule\n');
            end
        end
        
        fprintf(fid, '\\bottomrule\n');
        fprintf(fid, '\\end{tabular}\n');
        fprintf(fid, '\\end{table}\n\n');
    end
end

%% Section 5: Best and Worst Cases
fprintf(fid, '\\section{Extreme Cases Analysis}\n\n');

all_keys = fieldnames(results_database);

% Find best and worst by EE max error
ee_errors = zeros(length(all_keys), 1);
for i = 1:length(all_keys)
    ee_errors(i) = results_database.(all_keys{i}).metrics.ee_max_error_mm;
end

[~, best_idx] = min(ee_errors);
[~, worst_idx] = max(ee_errors);

fprintf(fid, '\\subsection{Best Performance}\n\n');
best_key = all_keys{best_idx};
best_meta = results_database.(best_key).metadata;
best_metrics = results_database.(best_key).metrics;

% ESCAPING UNDERSCORES IN METADATA STRINGS
traj_esc = strrep(best_meta.trajectory, '_', '\_');
matched_esc = strrep(best_meta.matched, '_', '\_');
optimized_esc = strrep(best_meta.optimized, '_', '\_');

fprintf(fid, '\\textbf{Configuration:}\n');
fprintf(fid, '\\begin{itemize}\n');
fprintf(fid, '  \\item Controller: %s\n', best_meta.controller);
fprintf(fid, '  \\item Uncertainty: %s\n', best_meta.uncertainty);
fprintf(fid, '  \\item Initial condition: %s\n', matched_esc);
fprintf(fid, '  \\item Gains: %s\n', optimized_esc);
fprintf(fid, '  \\item Trajectory: %s (label %s)\n', traj_esc, best_meta.label);
fprintf(fid, '\\end{itemize}\n\n');
fprintf(fid, '\\textbf{Metrics:}\n');
fprintf(fid, '\\begin{itemize}\n');
fprintf(fid, '  \\item EE max error: \\textcolor{bestcolor}{%.2f} mm\n', best_metrics.ee_max_error_mm);
fprintf(fid, '  \\item EE RMS error: %.2f mm\n', best_metrics.ee_rms_error_mm);
fprintf(fid, '  \\item Joint max error: %.2f deg\n', best_metrics.max_joint_error_deg);
fprintf(fid, '  \\item Joint RMS error: %.2f deg\n', best_metrics.rms_joint_error_deg);
fprintf(fid, '\\end{itemize}\n\n');

fprintf(fid, '\\subsection{Worst Performance}\n\n');
worst_key = all_keys{worst_idx};
worst_meta = results_database.(worst_key).metadata;
worst_metrics = results_database.(worst_key).metrics;

% ESCAPING UNDERSCORES IN METADATA STRINGS
traj_esc = strrep(worst_meta.trajectory, '_', '\_');
matched_esc = strrep(worst_meta.matched, '_', '\_');
optimized_esc = strrep(worst_meta.optimized, '_', '\_');

fprintf(fid, '\\textbf{Configuration:}\n');
fprintf(fid, '\\begin{itemize}\n');
fprintf(fid, '  \\item Controller: %s\n', worst_meta.controller);
fprintf(fid, '  \\item Uncertainty: %s\n', worst_meta.uncertainty);
fprintf(fid, '  \\item Initial condition: %s\n', matched_esc);
fprintf(fid, '  \\item Gains: %s\n', optimized_esc);
fprintf(fid, '  \\item Trajectory: %s (label %s)\n', traj_esc, worst_meta.label);
fprintf(fid, '\\end{itemize}\n\n');
fprintf(fid, '\\textbf{Metrics:}\n');
fprintf(fid, '\\begin{itemize}\n');
fprintf(fid, '  \\item EE max error: \\textcolor{worstcolor}{%.2f} mm\n', worst_metrics.ee_max_error_mm);
fprintf(fid, '  \\item EE RMS error: %.2f mm\n', worst_metrics.ee_rms_error_mm);
fprintf(fid, '  \\item Joint max error: %.2f deg\n', worst_metrics.max_joint_error_deg);
fprintf(fid, '  \\item Joint RMS error: %.2f deg\n', worst_metrics.rms_joint_error_deg);
fprintf(fid, '\\end{itemize}\n\n');

%% Section 6: Summary Table
fprintf(fid, '\\section{Summary Comparison Table}\n\n');
fprintf(fid, 'This section presents a condensed head-to-head comparison using End-Effector Max Error [mm]. The better value is \\textcolor{bestcolor}{Green}.\n\n');

% Count wins
fbl_wins = 0;
pbc_wins = 0;
total_comparisons = 0;

% Column specification: p{5cm} (Configuration wrapped), ccc (Metrics)
fprintf(fid, '\\begin{longtable}{p{5cm}ccc}\n'); 
fprintf(fid, '\\caption{Head-to-head controller comparison (lower is better)} \\\\\n');
fprintf(fid, '\\toprule\n');
% Simple, one-line headers:
fprintf(fid, 'Configuration & FBL EE Error [mm] & PBC EE Error [mm] & Difference [mm] \\\\\n');
fprintf(fid, '\\midrule\n');
fprintf(fid, '\\endfirsthead\n');
fprintf(fid, '\\toprule\n');
fprintf(fid, 'Configuration & FBL EE Error [mm] & PBC EE Error [mm] & Difference [mm] \\\\\n');
fprintf(fid, '\\midrule\n');
fprintf(fid, '\\endhead\n');

current_group = '';

for i = 1:length(all_keys)
    if startsWith(all_keys{i}, 'FBL_')
        fbl_key = all_keys{i};
        pbc_key = strrep(fbl_key, 'FBL_', 'PBC_');
        
        if isfield(results_database, pbc_key)
            total_comparisons = total_comparisons + 1;
            
            fbl_err = results_database.(fbl_key).metrics.ee_max_error_mm;
            pbc_err = results_database.(pbc_key).metrics.ee_max_error_mm;
            
            % --- Coloring Logic ---
            if fbl_err < pbc_err
                fbl_err_str = sprintf('\\textcolor{bestcolor}{%.2f}', fbl_err);
                pbc_err_str = sprintf('%.2f', pbc_err);
                winner_color_text = '\\textcolor{bestcolor}{FBL}'; % FIXED: Only define the colored text here
                fbl_wins = fbl_wins + 1;
                diff = pbc_err - fbl_err; 
            elseif pbc_err < fbl_err
                fbl_err_str = sprintf('%.2f', fbl_err);
                pbc_err_str = sprintf('\\textcolor{bestcolor}{%.2f}', pbc_err);
                winner_color_text = '\\textcolor{bestcolor}{PBC}'; % FIXED: Only define the colored text here
                pbc_wins = pbc_wins + 1;
                diff = fbl_err - pbc_err;
            else
                fbl_err_str = sprintf('%.2f', fbl_err);
                pbc_err_str = sprintf('%.2f', pbc_err);
                winner_color_text = 'Tie'; % FIXED: Only define the winner text here
                diff = 0;
            end
            
            % --- Configuration String Cleanup & Grouping ---
            meta = results_database.(fbl_key).metadata;
            traj_short = traj_map(meta.trajectory);
            config_short = config_map_short([meta.matched '_' meta.optimized]);
            
            % Group by Trajectory/Uncertainty (e.g., BCB/extreme)
            new_group = [traj_short '/' upper(meta.uncertainty)];
            if ~strcmp(new_group, current_group) && total_comparisons > 1
                 fprintf(fid, '\\midrule\n'); % New line to separate groups
            end
            current_group = new_group;
            
            % Full compact string: Traj/Unc/Match-Opt
            config_str = sprintf('\\textbf{%s}/%s/%s', traj_short, meta.uncertainty, config_short);
            config_str = strrep(config_str, '_', '\_'); % Escape underscores that may remain
            
            % Print row
            fprintf(fid, '%s (%s) & %s & %s & %.2f \\\\\n', ...
                    config_str, meta.label, fbl_err_str, pbc_err_str, diff); % Winner column removed
        end
    end
end

fprintf(fid, '\\bottomrule\n');
fprintf(fid, '\\end{longtable}\n\n');

fprintf(fid, '\\subsection{Overall Winner Statistics}\n\n');
if total_comparisons > 0
    fprintf(fid, '\\begin{itemize}\n');
    fprintf(fid, '  \\item FBL wins: %d (%.1f\\%%)\n', fbl_wins, 100*fbl_wins/total_comparisons);
    fprintf(fid, '  \\item PBC wins: %d (%.1f\\%%)\n', pbc_wins, 100*pbc_wins/total_comparisons);
    fprintf(fid, '  \\item Total comparisons: %d\n', total_comparisons);
    fprintf(fid, '\\end{itemize}\n\n');
end

%% Closing
fprintf(fid, '\\end{document}\n');
fclose(fid);

fprintf('\nLaTeX report generated successfully!\n');
fprintf('Output file: %s\n', output_file);
fprintf('\nTo compile the PDF, run:\n');
fprintf('  pdflatex %s\n', output_file);
fprintf('  pdflatex %s  (run twice for table of contents)\n\n', output_file);