function dx = robot_dynamics_bang_coast_bang(t, x, q0, qf, T_final, controller_func, controller_gains, true_params, uncertain_params, coast_fraction)
%ROBOT_DYNAMICS_BANG_COAST_BANG Dinamiche robot per traiettoria bang-coast-bang
%
% Input:
%   t - tempo corrente
%   x - stato [q; qd] [14x1]
%   q0, qf - configurazioni iniziale e finale
%   T_final - tempo totale traiettoria
%   controller_func - handle controller (@FBL_numeric)
%   controller_gains - gains controller
%   true_params - parametri VERI del plant (dinamica reale)
%   uncertain_params - parametri INCERTI per il controller
%   coast_fraction - frazione di tempo in coast (opzionale, default 0.4)
%
% Output:
%   dx - derivata stato [dq; dqd] [14x1]

% Default coast fraction
if nargin < 10
    coast_fraction = 0.4;
end

% Estrai stato corrente
q = x(1:7);     % posizioni attuali
qd = x(8:14);   % velocità attuali

% Genera traiettoria desiderata usando profilo bang-coast-bang
[q_des, qd_des, qdd_des] = generate_bang_coast_bang_trajectory(t, q0, qf, T_final, coast_fraction);

% Calcola controllo usando parametri incerti
tau = controller_func(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params);

% Calcola dinamiche plant usando parametri veri
[M, c, g, tau_f] = fast_dynamics(q, qd, true_params);

% Safety checks
% Controllo condizionamento matrice massa
if cond(M) > 1e12
    warning('robot_dynamics_bang_coast_bang: Matrice massa M mal condizionata (cond=%.2e)', cond(M));
    % Aggiungi regolarizzazione
    M = M + 1e-6 * eye(7);
end

% Controllo valori finiti nelle dinamiche
if any(~isfinite(c)) || any(~isfinite(g)) || any(~isfinite(tau_f))
    warning('robot_dynamics_bang_coast_bang: Dinamiche non finite, usando valori zero');
    c(~isfinite(c)) = 0;
    g(~isfinite(g)) = 0;
    tau_f(~isfinite(tau_f)) = 0;
end

% Controllo valori finiti nel controllo
if any(~isfinite(tau))
    warning('robot_dynamics_bang_coast_bang: Coppie controllo non finite, usando zero');
    tau(~isfinite(tau)) = 0;
end

% Equazione dinamica del robot: M*qdd + c + g + tau_f = tau
% Risolvi per qdd: qdd = M^(-1) * (tau - c - g - tau_f)
try
    qdd = M \ (tau - c - g - tau_f);
catch ME
    warning('robot_dynamics_bang_coast_bang: Errore soluzione dinamica');
    qdd = zeros(7,1);
end

% Limitazione accelerazioni per sicurezza
qdd_max = 50 * ones(7,1); % [rad/s^2] - limiti conservativi
qdd = max(-qdd_max, min(qdd_max, qdd));

% Safety: controllo valori finiti nell'accelerazione
if any(~isfinite(qdd))
    warning('robot_dynamics_bang_coast_bang: Accelerazioni non finite, usando zero');
    qdd(~isfinite(qdd)) = 0;
end

% Stato derivato
dx = [qd; qdd];

end