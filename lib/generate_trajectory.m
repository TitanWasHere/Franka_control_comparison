function [q_d, qd_d, qdd_d] = generate_trajectory(t, q0, qf, T_final)
    % ensure time is within [0, T_final]
    if t < 0
        t = 0;
    elseif t > T_final
        t = T_final;
    end
    
    delta_q = qf - q0;
    
    % coefficients for the quintic polynomial
    a0 = q0;
    a1 = zeros(size(q0));
    a2 = zeros(size(q0));
    a3 = (10 / T_final^3) * delta_q;
    a4 = -(15 / T_final^4) * delta_q;
    a5 = (6 / T_final^5) * delta_q;
    
    % compute desired position, velocity, and acceleration at time t
    q_d   = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    qd_d  =      a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    qdd_d =           2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;

end