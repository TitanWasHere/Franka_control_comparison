function [DH, masses, r_com, I_com] = get_robot_parameters(params_vec)
% get_robot_parameters - Unpacks a parameter vector into structured robot data.
%
% This function takes a single column vector containing all robot parameters 
% (masses, centers of mass, inertia tensors, etc.) and reshapes them into 
% formats usable by dynamics algorithms like RNEA.
%
% SYNTAX:
%   [DH, masses, r_com, I_com] = get_robot_parameters(params_vec)
%
% INPUTS:
%   params_vec - A single column vector containing the robot's dynamic
%                parameters stacked in a predefined order:
%                - 7 mass values
%                - 21 center of mass values (3x7 matrix, column-major)
%                - 63 inertia tensor values (3x21 matrix, column-major)
%
% OUTPUTS:
%   DH     - Denavit-Hartenberg parameters. Note: These are not included
%            in the provided params_vec, so an empty matrix is returned.
%   masses - (1x7) vector of link masses.
%   r_com  - (3x7) matrix where each column is the center of mass vector
%            for a link.
%   I_com  - (3x3x7) 3D array where each 3x3 slice is the inertia tensor
%            for a link.
%
%disp("palle")
% --- Validate Input ---
% A complete vector for a 7-link robot with masses, CoMs, and inertias 
% should have at least 7 + (3*7) + (9*7) = 91 elements.
if numel(params_vec) < 91
    error('params_vec does not contain enough elements for a 7-link robot (mass, CoM, inertia).');
end

% --- Denavit-Hartenberg (DH) Parameters ---
% The provided 'params_vec' does not contain kinematic DH parameters.
% These would need to be defined separately based on the robot's geometry.
% We return an empty matrix as a placeholder.
DH = get_DH();

% --- Unpack the Vector ---
currentIndex = 1;

% 1. Extract Masses (7 elements)
num_links = 7;
masses = params_vec(currentIndex : currentIndex + num_links - 1)'; % Transpose to a row vector
currentIndex = currentIndex + num_links;

% 2. Extract Centers of Mass (CoM) (21 elements)
com_elements = 3 * num_links;
r_com_flat = params_vec(currentIndex : currentIndex + com_elements - 1);
r_com = reshape(r_com_flat, [3, num_links]);
currentIndex = currentIndex + com_elements;

% 3. Extract Inertia Tensors (63 elements)
inertia_elements = 9 * num_links;
I_com_flat = params_vec(currentIndex : currentIndex + inertia_elements - 1);
% The inertia tensors are stacked as a 3x21 matrix, which we reshape into a 3x3x7 array
I_com_temp = reshape(I_com_flat, [3, 3 * num_links]);
I_com = zeros(3, 3, num_links);
for i = 1:num_links
    I_com(:,:,i) = I_com_temp(:, (i-1)*3 + 1 : i*3);
end


end