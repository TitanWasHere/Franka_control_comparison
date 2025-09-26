function tau_f = compute_friction(qd, params_vec)
    fv = params_vec(end-20 : end-14); % Viscous coefficients
    fc = params_vec(end-13 : end-7);  % Coulomb coefficients
    fo = params_vec(end-6 : end);     % Offset coefficients
    
    epsilon = 1e-3; % Small value for tanh steepness, equivalent to k=1000
    smooth_sign = tanh(qd / epsilon);
    
    % Full Friction Law: tau_f = F_v*qd + F_c*sign(qd) + F_o
    tau_f = fv .* qd + fc .* smooth_sign + fo;
end