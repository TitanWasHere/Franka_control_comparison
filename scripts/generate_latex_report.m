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
    
    fprintf(fid, '\\begin{table}[h]\n');
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

%% Section 3: Detailed Comparison Tables (FIXED COLUMN WIDTH)
fprintf(fid, '\\section{Detailed Performance Tables}\n\n');
fprintf(fid, 'Performance tables show FBL and PBC results on consecutive rows, with the better value (lower error) highlighted in \\textcolor{bestcolor}{Green}. The layout is optimized for head-to-head comparison.\n\n');

% Group by uncertainty level
uncertainty_levels = {'high', 'extreme'};
trajectories = {'quintic', 'bang_bang', 'bang_coast_bang'};

for u = 1:length(uncertainty_levels)
    unc = uncertainty_levels{u};
    fprintf(fid, '\\subsection{Uncertainty Level: %s}\n\n', upper(unc));
    
    for t = 1:length(trajectories)
        traj = trajectories{t};
        traj_name = traj_map(traj); % Use mapped name (Quintic, BB, BCB)
        
        % Escaped trajectory name for subsubsection title
        traj_name_esc = strrep(traj_name, '-', '\_');
        fprintf(fid, '\\subsubsection{Trajectory: %s}\n\n', traj_name_esc);
        
        % FIXED COLUMN WIDTH: Increased from p{1.5cm} to p{2.2cm} to prevent overflow
        fprintf(fid, '\\begin{longtable}{lp{2.2cm}cccc}\n');
        fprintf(fid, '\\caption{Performance comparison for %s trajectory with %s uncertainty} \\\\\n', traj_name_esc, unc);
        fprintf(fid, '\\toprule\n');
        % Simple, one-line headers:
        fprintf(fid, 'Controller & Config & EE Max [mm] & EE RMS [mm] & Jt Max [deg] & Jt RMS [deg] \\\\\n');
        fprintf(fid, '\\midrule\n');
        fprintf(fid, '\\endfirsthead\n');
        fprintf(fid, '\\multicolumn{6}{c}{{\\tablename\\ \\thetable{} -- continued}} \\\\\n');
        fprintf(fid, '\\toprule\n');
        fprintf(fid, 'Controller & Config & EE Max [mm] & EE RMS [mm] & Jt Max [deg] & Jt RMS [deg] \\\\\n');
        fprintf(fid, '\\midrule\n');
        fprintf(fid, '\\endhead\n');
        
        % Collect all matching results
        configs = {'matched_optimized', 'matched_unoptimized', 'mismatched_optimized', 'mismatched_unoptimized'};
        labels = {'A', 'B'};
        
        for cfg_idx = 1:length(configs)
            cfg = configs{cfg_idx};
            cfg_name = config_map(cfg); % Use mapped name (Match/Opt)
            
            for lbl_idx = 1:length(labels)
                lbl = labels{lbl_idx};
                
                % Find FBL and PBC results
                fbl_key = sprintf('FBL_%s_%s_%s_%s', unc, strrep(cfg, '_', '_'), traj, lbl);
                pbc_key = sprintf('PBC_%s_%s_%s_%s', unc, strrep(cfg, '_', '_'), traj, lbl);
                
                if isfield(results_database, fbl_key) && isfield(results_database, pbc_key)
                    fbl_m = results_database.(fbl_key).metrics;
                    pbc_m = results_database.(pbc_key).metrics;
                    
                    config_label = [cfg_name '-' lbl];
                    
                    % --- Coloring Logic (Lower is Better) ---
                    metrics_to_compare = {'ee_max_error_mm', 'ee_rms_error_mm', 'max_joint_error_deg', 'rms_joint_error_deg'};
                    fbl_str = cell(1, 4);
                    pbc_str = cell(1, 4);
                    
                    for m_idx = 1:length(metrics_to_compare)
                        metric = metrics_to_compare{m_idx};
                        fbl_val = fbl_m.(metric);
                        pbc_val = pbc_m.(metric);
                        
                        if fbl_val < pbc_val
                            fbl_str{m_idx} = sprintf('\\textcolor{bestcolor}{%.2f}', fbl_val);
                            pbc_str{m_idx} = sprintf('%.2f', pbc_val);
                        elseif pbc_val < fbl_val
                            fbl_str{m_idx} = sprintf('%.2f', fbl_val);
                            pbc_str{m_idx} = sprintf('\\textcolor{bestcolor}{%.2f}', pbc_val);
                        else
                            fbl_str{m_idx} = sprintf('%.2f', fbl_val);
                            pbc_str{m_idx} = sprintf('%.2f', pbc_val);
                        end
                    end
                    
                    % --- Print FBL and PBC rows (Config only on FBL row) ---
                    fprintf(fid, 'FBL & %s & %s & %s & %s & %s \\\\\n', ...
                            config_label, fbl_str{1}, fbl_str{2}, fbl_str{3}, fbl_str{4});
                    fprintf(fid, 'PBC & & %s & %s & %s & %s \\\\\n', ...
                            pbc_str{1}, pbc_str{2}, pbc_str{3}, pbc_str{4});
                            
                    % --- Print Winner Row (based on EE Max Error) ---
                    ee_diff = fbl_m.ee_max_error_mm - pbc_m.ee_max_error_mm;
                    if ee_diff > 0
                        winner_text = sprintf('\\textcolor{bestcolor}{PBC}'); % PBC error is smaller
                        ee_diff_abs = ee_diff;
                    elseif ee_diff < 0
                        winner_text = sprintf('\\textcolor{bestcolor}{FBL}'); % FBL error is smaller
                        ee_diff_abs = -ee_diff;
                    else
                         winner_text = sprintf('Tie');
                        ee_diff_abs = 0;
                    end
                    
                    fprintf(fid, '\\midrule\n');
                    fprintf(fid, '\\multicolumn{6}{l}{\\textit{Winner: %s (EE Diff: %.2f mm)}} \\\\\n', winner_text, ee_diff_abs);
                    
                    % Use \cmidrule for clear grouping
                    if ~(cfg_idx == length(configs) && lbl_idx == length(labels))
                        fprintf(fid, '\\cmidrule{1-6}\n'); % Horizontal line across all columns
                    end
                end
            end
        end
        
        fprintf(fid, '\\bottomrule\n');
        fprintf(fid, '\\end{longtable}\n\n');
    end
end

%% Section 4: Best and Worst Cases
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

%% Section 5: Head-to-head comparisons (FIXED WINNER STRING)
fprintf(fid, '\\section{Direct Controller Comparisons}\n\n');
fprintf(fid, 'This section presents head-to-head comparisons between FBL and PBC controllers under identical conditions, using the End-Effector Max Error [mm] metric. The better value is \\textcolor{bestcolor}{Green}. The configuration is abbreviated to *Profile/Unc/Match-Opt/traj*.\n\n');

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