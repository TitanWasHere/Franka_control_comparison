clear; clc; close all;

% Add lib to path
try
    this_script_path = mfilename('fullpath');
    [script_dir, ~, ~] = fileparts(this_script_path);
    [project_root, ~, ~] = fileparts(script_dir);
    addpath(fullfile(project_root, 'lib'));
catch
    warning('Could not determine paths automatically.');
end

fprintf('=== REGENERATING PLOTS FROM SAVED RESULTS ===\n');

uncertainties = ["high", "extreme"]; 
controllers = ["FBL", "PBC"];
trajectories = ["quintic", "bang_bang", "bang_coast_bang"];
labels = ['A', 'B'];
opts = [true, false];       % Optimized gains
matches = [true, false];    % Matched start

% Loop through all combinations
for unc = uncertainties
    for ctrl = controllers
        for traj = trajectories
            for lbl = labels
                for opt = opts
                    for match = matches
                        
                        % 1. Reconstruct the directory path
                        % Structure: results/CONTROLLER/UNCERTAINTY/MATCHED/OPTIMIZED/TRAJ_TYPE/
                        
                        % Match string
                        if match, match_str = "matched"; else, match_str = "mismatched"; end
                        
                        % Opt string
                        if opt, opt_str = "optimized"; else, opt_str = "unoptimized"; end
                        
                        % Build folder path
                        sub_path = fullfile('results', ctrl, unc, match_str, opt_str, traj);
                        data_dir = fullfile(project_root, sub_path);
                        
                        % 2. Reconstruct filename
                        % Suffix logic from setup_simulation
                        switch traj
                            case "quintic", suffix = "quintic";
                            case "bang_bang", suffix = "bb";
                            case "bang_coast_bang", suffix = "bcb";
                        end
                        filename = sprintf("%s_%s_%s.mat", ctrl, suffix, lbl);
                        full_path = fullfile(data_dir, filename);
                        
                        % 3. Load and Process
                        if exist(full_path, 'file')
                            fprintf('Processing: %s... ', filename);
                            
                            try
                                % Load the saved struct
                                data = load(full_path);
                                res = data.results;
                                
                                % Reconstruct x_sim (State vector)
                                % x_sim must be [N x 14] for process_and_save_results
                                q = res.simulation.q';   % Transpose to [N x 7]
                                qd = res.simulation.qd'; % Transpose to [N x 7]
                                x_sim = [q, qd]; 
                                t_sim = res.simulation.t;
                                sim_time = res.simulation_time;
                                
                                if ~isfield(res.config, 'controller_func')
                                    if ctrl == "FBL", res.config.controller_func = @FBL;
                                    else, res.config.controller_func = @PBC; end
                                end
                                
                                res.config.resultsPath = full_path;
                                res.config.plot_dir = fullfile(project_root, 'plots', ...
                                    ctrl, unc, match_str, opt_str, traj);
                                
                                process_and_save_results(res.config, t_sim, x_sim, sim_time);
                                
                                fprintf('Done.\n');
                                
                            catch ME
                                fprintf('FAILED.\nError: %s\n', ME.message);
                            end
                        else
                            % Silent skip if file doesn't exist
                        end
                        
                    end
                end
            end
        end
    end
end

fprintf('\nAll plots regenerated.\n');