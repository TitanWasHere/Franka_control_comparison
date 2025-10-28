function [q, qd, qdd] = generate_bang_coast_bang_trajectory(t, q0, qf, T_final, coast_fraction)
    % phase 1: constant acceleration
    % phase 2: constant velocity (coast)
    % phase 3: constant deceleration

    % default coast fraction
    if nargin < 5
        coast_fraction = 0.4;
    end
    
    % limit coast fraction between 10% and 80%
    coast_fraction = max(0.1, min(0.8, coast_fraction));
    
    % init output
    q = zeros(7, 1);
    qd = zeros(7, 1);
    qdd = zeros(7, 1);
    
    % normalized time [0, 1]
    tau = max(0, min(1, t / T_final));
    
    % compute time fractions
    tau_coast = coast_fraction;
    tau_accel = (1 - tau_coast) / 2;
    tau_decel = tau_accel;
    
    % transition normalized times
    tau1 = tau_accel;  % end acceleration, start coast
    tau2 = tau_accel + tau_coast;  % end coast start deceleration
    
    % absolute times
    T_accel = tau_accel * T_final;
    T_coast = tau_coast * T_final;
    T_decel = tau_decel * T_final;
    
    % compute an independent bang-coast-bang trajectory for each joint
    for i = 1:7
        Delta_q = qf(i) - q0(i); % total displacement required
        
        % if no displacement, keep constant position
        if abs(Delta_q) < 1e-6
            q(i) = q0(i);
            qd(i) = 0;
            qdd(i) = 0;
            continue;
        end
        
        % Delta_q = 0.5*a*T_accel^2 + a*T_accel*T_coast + 0.5*a*T_accel^2
        % Delta_q = a*T_accel*(T_accel + T_coast)
        % v_const = a*T_accel
        
        a_max = Delta_q / (T_accel * (T_accel + T_coast)); % required acceleration
        v_const = a_max * T_accel; % constant velocity during coast
        
        if tau <= tau1
            % phase 1: acceleration (0 <= tau <= tau1)
            t_local = tau * T_final;
            
            q(i) = q0(i) + 0.5 * a_max * t_local^2;
            qd(i) = a_max * t_local;
            qdd(i) = a_max;
            
        elseif tau <= tau2
            % phase 2: coast - constant velocity (tau1 < tau <= tau2)
            t_local = tau * T_final;
            t_coast_local = t_local - T_accel;
            
            % position at the end of acceleration
            q_end_accel = q0(i) + 0.5 * a_max * T_accel^2;
            
            % position during coast
            q(i) = q_end_accel + v_const * t_coast_local;
            qd(i) = v_const;
            qdd(i) = 0;
            
        else
            % phase 3: deceleration (tau2 < tau <= 1)
            t_local = tau * T_final;
            t_decel_local = t_local - T_accel - T_coast;
            
            % position at the end of coast
            q_end_coast = q0(i) + 0.5 * a_max * T_accel^2 + v_const * T_coast;
            
            % position during deceleration
            q(i) = q_end_coast + v_const * t_decel_local - 0.5 * a_max * t_decel_local^2;
            qd(i) = v_const - a_max * t_decel_local;
            qdd(i) = -a_max;
        end
    end
    
end