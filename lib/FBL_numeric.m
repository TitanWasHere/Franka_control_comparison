function tau = FBL_numeric(q, qd, q_des, qd_des, qdd_des, controller_gains, uncertain_params)
%FBL_NUMERIC Feedback Linearization controller usando dinamiche numeriche
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

% Estrai gains
Kp = controller_gains.Kp;
Kd = controller_gains.Kd;

% Calcola dinamiche usando PARAMETRI INCERTI
[M, c, g, tau_f] = fast_dynamics(q, qd, uncertain_params);

% Errori di tracking
e_pos = q_des - q;      % errore posizione
e_vel = qd_des - qd;    % errore velocità

% Legge di controllo FBL CORRETTA
v = qdd_des + Kp * e_pos + Kd * e_vel;   % input ausiliario completo

% SAFETY: Limita input ausiliario per evitare esplosioni
v_max = 10 * ones(7,1); % [rad/s^2] 
v = max(-v_max, min(v_max, v));

% Coppie controllo
tau = M * v + c + g + tau_f;

% SAFETY: Controllo NaN/Inf
if any(~isfinite(tau))
    warning('FBL_numeric: tau non finito, usando zero');
    tau = zeros(7,1);
end

% Saturazione coppie (limiti Franka Panda)
tau_max = [87, 87, 87, 87, 12, 12, 12]'; % [Nm]
tau = max(-tau_max, min(tau_max, tau));

end