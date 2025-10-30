clear all; clc; close all;

addpath("../lib/")

all_controller_types = "FBL"; %["FBL", "PBC"];
all_trajectory_types = ["quintic", "bang_bang", "bang_coast_bang"];
all_traj_labels = ['A', 'B'];
matched = [true, false];
for controller_type = all_controller_types
    for trajectory_type = all_trajectory_types
        for traj_label = all_traj_labels
            for match = matached
                fprintf('Starting simulation: Controller=%s, Trajectory=%s (%s)\n', ...
                        controller_type, trajectory_type, traj_label);
                start_sim(controller_type, trajectory_type, traj_label, match, false, 8.0);
            end
        end
    end
end