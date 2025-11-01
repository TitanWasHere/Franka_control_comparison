function uncertainty_levels = setup_uncertainty_levels(true_params)
    % set up different levels of parameter uncertainty.
    uncertainty_levels = struct();
    
    uncertainty_levels.friction_only = create_uncertain_params(true_params, 'friction', 1.20);
    uncertainty_levels.low = create_uncertain_params(true_params, 'low', 1.20);
    uncertainty_levels.mid = create_uncertain_params(true_params, 'mid', 1.20);
    uncertainty_levels.high = create_uncertain_params(true_params, 'high', 1.20);
    uncertainty_levels.extreme = create_uncertain_params(true_params, 'extreme', 1.20);
end

function uncertain_params = create_uncertain_params(true_params, uncertainty_type, factor)
    uncertain_params = true_params;
    
    switch uncertainty_type
        case 'extreme'
            uncertain_params = true_params * factor;

        case 'high'
            uncertain_params(1) = true_params(1) * factor;    % m1
            uncertain_params(2) = true_params(2) * factor;    % m2
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(4) = true_params(4) * factor;    % m4
            uncertain_params(5) = true_params(5) * factor;    % m5
            uncertain_params(6) = true_params(6) * factor;    % m6
            uncertain_params(7) = true_params(7) * factor;    % m7

            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc

        case 'mid'
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5  
            uncertain_params(7) = true_params(7) * factor;    % m7
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
            
        case 'low'
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5
            uncertain_params(7) = true_params(7) * factor;    % m7
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            
        case 'friction'
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
    end
end

