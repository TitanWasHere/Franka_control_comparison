function [q, qd, qdd] = generate_bang_bang_trajectory(t, q0, qf, T_final)
%GENERATE_BANG_BANG_TRAJECTORY Genera traiettoria rest-to-rest con profilo bang-bang
%
% Input:
%   t - tempo corrente [s]
%   q0 - configurazione iniziale [7x1] [rad]
%   qf - configurazione finale [7x1] [rad]
%   T_final - tempo totale [s]
%
% Output:
%   q - posizione desiderata [7x1] [rad]
%   qd - velocità desiderata [7x1] [rad/s]
%   qdd - accelerazione desiderata [7x1] [rad/s²]
%
% Profilo bang-bang:
% - Prima metà: accelerazione costante massima
% - Seconda metà: decelerazione costante massima
% - Tempo di switch: T_final/2
% - Velocità massima raggiunta a metà traiettoria

% Inizializzazione output
q = zeros(7, 1);
qd = zeros(7, 1);
qdd = zeros(7, 1);

% Tempo normalizzato [0, 1]
tau = max(0, min(1, t / T_final));

% Tempo di switch (metà del tempo totale)
tau_switch = 0.5;
T_switch = T_final * tau_switch;

% Per ogni joint, calcola traiettoria bang-bang indipendente
for i = 1:7
    % Spostamento totale richiesto
    Delta_q = qf(i) - q0(i);
    
    % Se lo spostamento è zero, mantieni posizione costante
    if abs(Delta_q) < 1e-6
        q(i) = q0(i);
        qd(i) = 0;
        qdd(i) = 0;
        continue;
    end
    
    % Calcolo parametri bang-bang
    % Accelerazione massima necessaria per completare il movimento
    % Delta_q = 0.5 * a_max * (T_switch)^2 + a_max * T_switch * (T_final - T_switch) - 0.5 * a_max * (T_final - T_switch)^2
    % Semplificando per T_switch = T_final/2:
    % Delta_q = 0.25 * a_max * T_final^2
    a_max = 4 * Delta_q / (T_final^2);
    
    % Velocità massima raggiunta a metà tempo
    v_max = a_max * T_switch;
    
    if tau <= tau_switch
        % Prima fase: accelerazione costante (0 <= t <= T_switch)
        t_local = tau * T_final;
        
        q(i) = q0(i) + 0.5 * a_max * t_local^2;
        qd(i) = a_max * t_local;
        qdd(i) = a_max;
        
    else
        % Seconda fase: decelerazione costante (T_switch < t <= T_final)
        t_local = tau * T_final;
        t_decel = t_local - T_switch;
        
        % Posizione: somma della fase di accelerazione + fase di decelerazione
        q_switch = q0(i) + 0.5 * a_max * T_switch^2;  % Posizione a metà tempo
        q(i) = q_switch + v_max * t_decel - 0.5 * a_max * t_decel^2;
        
        % Velocità: velocità massima meno decelerazione
        qd(i) = v_max - a_max * t_decel;
        
        % Accelerazione: costante negativa
        qdd(i) = -a_max;
    end
end

% Verifica consistenza: alla fine dovremmo avere velocità zero
if abs(tau - 1.0) < 1e-6
    qd = zeros(7, 1);  % Assicura velocità zero alla fine
end

end