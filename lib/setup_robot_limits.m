function limits = setup_robot_limits()
    % physical limits for the Franka Emika Panda robot
    
    % torque limits [Nm]
    limits.tau_max = [87, 87, 87, 87, 12, 12, 12]';
    limits.tau_min = -limits.tau_max;
    
    % position limits [rad]
    limits.q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]';
    limits.q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]';
    
    % velocity limits [rad/s]
    limits.qd_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]';
    limits.qd_min = -limits.qd_max;
    
    % safety margin for position limits (used when defining trajectory endpoints)
    limits.safety_margin = 0.1;
end