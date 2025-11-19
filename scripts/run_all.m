clear all; clc; close all;

addpath("../lib/")

uncertainties = ["high", "extreme"]; %["friction", "low", "mid", "high", "extreme"] 
all_controller_types = ["FBL"]; %["FBL", "PBC"];
all_trajectory_types = ["quintic", "bang_bang", "bang_coast_bang"];
all_traj_labels = ['A', 'B'];
optimized = [true, false];
matched = [true, false];

n = numel(uncertainties) * numel(all_controller_types) * numel(all_trajectory_types) * numel(all_traj_labels) * numel(optimized) * numel(matched);
i = 1;
for uncertainty = uncertainties
    for controller_type = all_controller_types
        for trajectory_type = all_trajectory_types
            for traj_label = all_traj_labels
                for opt = optimized
                    for match = matched
                        fprintf('[%d/%d]Starting simulation: Controller=%s, Trajectory=%s (%s)\n', ...
                                i,n,controller_type, trajectory_type, traj_label);
                        start_sim(controller_type, trajectory_type, traj_label, match, opt, 8.0, uncertainty);
                        i = i+1;
                    end
                end
            end
        end
    end
end