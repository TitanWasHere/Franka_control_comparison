function uncertainty_levels = setup_uncertainty_levels(true_params)
    % set up different levels of parameter uncertainty.
    uncertainty_levels = struct();
    
    % A.1) full uncertainty applied to masses, inertias, and friction
    uncertainty_levels.full = create_uncertain_params(true_params, 'full', 1.20);
    
    % A.2) only in inertia and Coriolis terms (masses/inertias)
    uncertainty_levels.inertia_coriolis = create_uncertain_params(true_params, 'inertia_coriolis', 1.20);
    
    % A.3) only in friction terms
    uncertainty_levels.friction_only = create_uncertain_params(true_params, 'friction', 1.20);
end

function uncertain_params = create_uncertain_params(true_params, uncertainty_type, factor)
    uncertain_params = true_params;
    
    switch uncertainty_type
        case 'full'
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5  
            uncertain_params(7) = true_params(7) * factor;    % m7
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
            
        case 'inertia_coriolis'
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5
            uncertain_params(7) = true_params(7) * factor;    % m7
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            
        case 'friction'
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
    end
end

