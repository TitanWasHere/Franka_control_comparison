function [M, c, g] = recursive_newton_euler(q, qd, qdd, params_vec)
    [DH, masses, r_com, I_com] = get_robot_parameters(params_vec);
    n = 7;
    g0 = [0; 0; -9.81];

    % get M
    M = zeros(n, n);
    for j = 1:n
        qdd_unit = zeros(n, 1);
        qdd_unit(j) = 1;
        M(:, j) = newton_euler_forward(q, zeros(n,1), qdd_unit, DH, masses, r_com, I_com, [0;0;0]);
    end

    % get g
    g = newton_euler_forward(q, zeros(n,1), zeros(n,1), DH, masses, r_com, I_com, g0);

    % get coriolis vector
    c = newton_euler_forward(q, qd, zeros(n,1), DH, masses, r_com, I_com, [0;0;0]);
end

function tau = newton_euler_forward(q, qd, qdd, DH, masses, r_com, I_com, g0)
    n = length(q);
    tau = zeros(n, 1);
    
    % init
    w = zeros(3, n + 1);
    w_dot = zeros(3, n + 1);
    v_dot = zeros(3, n + 1);
    v_dot(:,1) = -g0;
    
    R = cell(1, n); % store rotation matrices
    p = cell(1, n); % store position vectors

    % forward recursion (base to ee)
    for i = 1:n
        alpha = DH(i, 2); a = DH(i, 1); d = DH(i, 3); theta = q(i) + DH(i, 4);
        
        T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1];
        
        R{i} = T_i(1:3, 1:3);
        p{i} = T_i(1:3, 4);
        z_axis = [0; 0; 1];

        w(:, i+1)     = R{i}' * w(:, i) + qd(i) * z_axis;
        w_dot(:, i+1) = R{i}' * w_dot(:, i) + cross(R{i}' * w(:, i), qd(i) * z_axis) + qdd(i) * z_axis;
        v_dot(:, i+1) = R{i}' * (v_dot(:, i) + cross(w_dot(:, i), p{i}) + cross(w(:, i), cross(w(:, i), p{i})));
    end

    % backward recursion (ee to base)
    f = zeros(3, n + 1);  % forces
    mu = zeros(3, n + 1); % moments

    for i = n:-1:1
        % acceleration of the CoM for link i
        vc_dot = v_dot(:, i+1) + cross(w_dot(:, i+1), r_com(:, i)) + cross(w(:, i+1), cross(w(:, i+1), r_com(:, i)));

        % net force and moment on link i
        F_i = masses(i) * vc_dot;
        N_i = I_com(:,:,i) * w_dot(:, i+1) + cross(w(:, i+1), I_com(:,:,i) * w(:, i+1));

        % propagate forces and moments from link i+1 to link i
        if i == n
            f(:, i) = F_i;
            mu(:, i) = N_i + cross(r_com(:, i), F_i);
        else
            f(:, i)  = R{i+1} * f(:, i+1) + F_i;
            mu(:, i) = N_i + R{i+1} * mu(:, i+1) + cross(r_com(:, i), F_i) + cross(p{i+1}, R{i+1} * f(:, i+1));
        end
        
        % project moment onto the joint axis to find the torque
        tau(i) = mu(:, i)' * [0; 0; 1];
    end
end