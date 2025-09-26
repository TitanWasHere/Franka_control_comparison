function dx = robot_dynamics_numeric(t, x, q0, qf, T_final, controller_func, controller_gains, true_params, uncertain_params)
%ROBOT_DYNAMICS_NUMERIC 
%
% Input:
%   t - tempo corrente
%   x - stato [q; qd] [14x1]
%   q0, qf, T_final - parametri traiettoria  
%   controller_func - handle controller (@FBL_numeric)
%   controller_gains - gains controller
%   true_params - parametri VERI del plant (dinamica reale)
%   uncertain_params - parametri INCERTI per il controller
%
% Output:
%   dx - derivata stato [dq; dqd] [14x1]

% Estrai stato
q = x(1:7);     % posizioni
qd = x(8:14);   % velocità

% Genera traiettoria desiderata
[q_des, qd_des, qdd_des] = generate_trajectory(t, q0, qf, T_final);

% Calcola controllo usando PARAMETRI INCERTI
tau = controller_func(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params);

% Calcola dinamiche plant usando PARAMETRI VERI
[M, c, g, tau_f] = fast_dynamics(q, qd, true_params);

% SAFETY: Controllo condizionamento matrice M
if cond(M) > 1e12
    warning('robot_dynamics_numeric: M mal condizionata, usando identità');
    M = diag([1, 1, 1, 1, 0.5, 0.5, 0.2]); % Matrice sicura
end

% SAFETY: Controllo valori finiti
if any(~isfinite(c)) || any(~isfinite(g)) || any(~isfinite(tau_f))
    warning('robot_dynamics_numeric: dinamiche non finite, usando zero');
    c = zeros(7,1);
    g = zeros(7,1); 
    tau_f = zeros(7,1);
end

% Equazione dinamica: M*qdd + c + g + tau_f = tau
% Risolvi per qdd: qdd = M^(-1) * (tau - c - g - tau_f)
qdd = M \ (tau - c - g - tau_f);

% SAFETY: Limita accelerazioni
qdd_max = 50 * ones(7,1); % [rad/s^2]
qdd = max(-qdd_max, min(qdd_max, qdd));

% Stato derivato
dx = [qd; qdd];

end