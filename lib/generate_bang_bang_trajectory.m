function [q, qd, qdd] = generate_bang_bang_trajectory(t, q0, qf, T_final)
    % first (second) half = constant maximum acceleration (deceleration)
    % switch time = T_final/2; v_max reached halfway.
    
    % init output
    q = zeros(7, 1);
    qd = zeros(7, 1);
    qdd = zeros(7, 1);
    
    % normalized time [0,1]
    tau = max(0, min(1, t / T_final));
    
    % switch time
    tau_switch = 0.5;
    T_switch = T_final * tau_switch;
    
    % compute an independent bang-bang trajectory for each joint
    for i = 1:7
        Delta_q = qf(i) - q0(i); % total displacement required
        
        % if no displacement, keep constant position
        if abs(Delta_q) < 1e-6
            q(i) = q0(i);
            qd(i) = 0;
            qdd(i) = 0;
            continue;
        end
        
        % Delta_q = 0.5 * a_max * (T_switch)^2 + a_max * T_switch * (T_final - T_switch) - 0.5 * a_max * (T_final - T_switch)^2
        % Delta_q = 0.25 * a_max * T_final^2
        a_max = 4 * Delta_q / (T_final^2); % required max acceleration
        v_max = a_max * T_switch; % max velocity reached at half time
        
        if tau <= tau_switch
            % phase 1: constant acceleration (0 <= t <= T_switch)
            t_local = tau * T_final;
            
            q(i) = q0(i) + 0.5 * a_max * t_local^2;
            qd(i) = a_max * t_local;
            qdd(i) = a_max;
            
        else
            % phase 2: constant deceleration (T_switch < t <= T_final)
            t_local = tau * T_final;
            t_decel = t_local - T_switch;
            
            % position =  phase 1 + phase 2
            q_switch = q0(i) + 0.5 * a_max * T_switch^2;  % position at half time
            q(i) = q_switch + v_max * t_decel - 0.5 * a_max * t_decel^2;
            
            % velocity = max velocity minus deceleration
            qd(i) = v_max - a_max * t_decel;
            
            % acceleration: constant negative value
            qdd(i) = -a_max;
        end
    end
end