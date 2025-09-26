function limits = setup_robot_limits()
    % Limiti fisici del robot Franka Emika Panda
    
    % Limiti di coppia [Nm]
    limits.tau_max = [87, 87, 87, 87, 12, 12, 12]';
    limits.tau_min = -limits.tau_max;
    
    % Limiti di posizione [rad]
    limits.q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]';
    limits.q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]';
    
    % Limiti di velocità [rad/s]
    limits.qd_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]';
    limits.qd_min = -limits.qd_max;
    
    % Margine di sicurezza per i limiti di posizione
    limits.safety_margin = 0.1; % 0.1 rad = ~5.7°
end