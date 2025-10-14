function tau = FBL_stable(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params)
%FBL_STABLE Feedback Linearization con inversione numerica avanzata
%
% Input:
%   q - configurazione attuale [7x1]
%   qd - velocità attuale [7x1]
%   q_des - posizione desiderata [7x1]  
%   qd_des - velocità desiderata [7x1]
%   qdd_des - accelerazione desiderata [7x1]
%   controller_gains - gains controller (Kp, Kd)
%   uncertain_params - parametri INCERTI per il controller
%
% Output:
%   tau - coppie controllo [7x1]

persistent stability_counter;
if isempty(stability_counter)
    stability_counter = 0;
    fprintf('[FBL_STABLE] Controller numericamente stabile inizializzato\n');
end

% Estrai gains
Kp = controller_gains.Kp;
Kd = controller_gains.Kd;

[M, c, g, tau_f] = fast_dynamics(q, qd, uncertain_params);

e_pos = q_des - q;      % errore posizione
e_vel = qd_des - qd;    % errore velocità

% Input ausiliario
v = qdd_des + Kp * e_pos + Kd * e_vel;

% SAFETY: Limita input ausiliario
v_max = 10 * ones(7,1); % [rad/s^2] 
v = max(-v_max, min(v_max, v));

cond_M = cond(M);

if cond_M < 1e10
    method = 'direct';
    tau = M * v + c + g + tau_f;
    
elseif cond_M < 1e15
    method = 'regularized';
    lambda = 1e-3 * trace(M) / 7;  
    M_reg = M + lambda * eye(7);
    tau = M_reg * v + c + g + tau_f;
    
else
    method = 'pseudoinv_pd';

    M_pinv = pinv(M, 1e-6);  % Tolleranza più rilassata
    tau_fbl = M_pinv \ (v - M_pinv * (c + g + tau_f));
        
    % Verifica risultato
    if any(~isfinite(tau_fbl)) || norm(tau_fbl) > 1000
        error('Pseudo-inversa instabile');
    end
        
    tau = tau_fbl + c + g + tau_f;
        

end

% ===== MONITORING STABILITÀ =====
stability_counter = stability_counter + 1;

% Controllo NaN/Inf
if any(~isfinite(tau))
    warning('FBL_stable: tau non finito, usando controllo PD di emergenza');
    tau = Kp * e_pos + Kd * e_vel;
    
    % Assicurati che anche il PD sia finito
    if any(~isfinite(tau))
        tau = zeros(7,1);
    end
end

% Saturazione coppie (limiti Franka Panda)
tau_max = [87, 87, 87, 87, 12, 12, 12]'; % [Nm]
tau = max(-tau_max, min(tau_max, tau));

if any(~isfinite(tau))
    warning('FBL_stable: tau finale non finito, usando zero');
    tau = zeros(7,1);
end


end