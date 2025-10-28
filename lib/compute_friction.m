function tau_f = compute_friction(qd, params_vec)
    fv = params_vec(end-20 : end-14); % viscous coefficients
    fc = params_vec(end-13 : end-7);  % Coulomb coefficients
    fo = params_vec(end-6 : end);     % offset coefficients
    
    % smoothing parameter 
    epsilon = 1e-3; % small value for smooth transition
    smooth_sign = tanh(qd / epsilon);
    
    % full friction: tau_f = F_v*qd + F_c*tanh(qd/epsilon) + F_o
    tau_f = fv .* qd + fc .* smooth_sign + fo;
end