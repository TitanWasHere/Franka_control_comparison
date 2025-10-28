function [DH, masses, r_com, I_com] = get_robot_parameters(params_vec)
    % extract all robot parameters
    % params_vec: 7 mass values, 21 CoMs, 63 inertia tensors

    if numel(params_vec) < 91
        error('params_vec does not contain enough elements for a 7-link robot (mass, CoM, inertia).');
    end

    DH = get_DH();

    % index to unpack
    currentIndex = 1;

    % extract masses (7 elements)
    num_links = 7;
    masses = params_vec(currentIndex : currentIndex + num_links - 1)';
    currentIndex = currentIndex + num_links;

    % extract CoMs (21 elements)
    com_elements = 3 * num_links;
    r_com_flat = params_vec(currentIndex : currentIndex + com_elements - 1);
    r_com = reshape(r_com_flat, [3, num_links]);
    currentIndex = currentIndex + com_elements;

    % extract inertia tensors (63 elements)
    inertia_elements = 9 * num_links;
    I_com_flat = params_vec(currentIndex : currentIndex + inertia_elements - 1);

    % inertia tensors 3x21 matrix => reshape into a 3x3x7 array
    I_com_temp = reshape(I_com_flat, [3, 3 * num_links]);
    I_com = zeros(3, 3, num_links);
    for i = 1:num_links
        I_com(:,:,i) = I_com_temp(:, (i-1)*3 + 1 : i*3);
    end
end