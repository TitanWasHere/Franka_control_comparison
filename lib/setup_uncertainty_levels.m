function uncertainty_levels = setup_uncertainty_levels(true_params)
    % Setup dei diversi livelli di incertezza parametrica
    
    uncertainty_levels = struct();
    
    % A.1) Tutti i parametri con incertezza
    uncertainty_levels.full = create_uncertain_params(true_params, 'full', 1.20);
    
    % A.2) Solo Coriolis e matrice di inerzia
    uncertainty_levels.inertia_coriolis = create_uncertain_params(true_params, 'inertia_coriolis', 1.20);
    
    % A.3) Solo attrito
    uncertainty_levels.friction_only = create_uncertain_params(true_params, 'friction', 1.20);
end

function uncertain_params = create_uncertain_params(true_params, uncertainty_type, factor)
    % Crea parametri incerti basati sul tipo di incertezza
    
    uncertain_params = true_params; % Inizia con parametri veri
    
    switch uncertainty_type
        case 'full'
            % Incertezza su masse, inerzie e attrito
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5  
            uncertain_params(7) = true_params(7) * factor;    % m7
            
            % Inerzie (elementi selezionati)
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            
            % Attrito
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
            
        case 'inertia_coriolis'
            % Solo masse e inerzie (influenzano M e C)
            uncertain_params(3) = true_params(3) * factor;    % m3
            uncertain_params(5) = true_params(5) * factor;    % m5
            uncertain_params(7) = true_params(7) * factor;    % m7
            uncertain_params(91) = true_params(91) * factor;  % Ic7zz
            
        case 'friction'
            % Solo attrito
            uncertain_params(end-13:end-7) = true_params(end-13:end-7) * factor; % fv
            uncertain_params(end-6:end) = true_params(end-6:end) * factor;       % fc
    end
end

