function [q, qd, qdd] = generate_bang_coast_bang_trajectory(t, q0, qf, T_final, coast_fraction)
%GENERATE_BANG_COAST_BANG_TRAJECTORY Genera traiettoria rest-to-rest con profilo bang-coast-bang
%
% Input:
%   t - tempo corrente [s]
%   q0 - configurazione iniziale [7x1] [rad]
%   qf - configurazione finale [7x1] [rad]
%   T_final - tempo totale [s]
%   coast_fraction - frazione di tempo in coast (default: 0.4, ovvero 40% del tempo totale)
%
% Output:
%   q - posizione desiderata [7x1] [rad]
%   qd - velocità desiderata [7x1] [rad/s]
%   qdd - accelerazione desiderata [7x1] [rad/s²]
%
% Profilo bang-coast-bang:
% - Prima fase: accelerazione costante (bang-up)
% - Seconda fase: velocità costante (coast)
% - Terza fase: decelerazione costante (bang-down)
% - Simmetrico: tempo accelerazione = tempo decelerazione

% Default coast fraction se non specificato
if nargin < 5
    coast_fraction = 0.4;  % 40% del tempo in coast
end

% Validazione input
coast_fraction = max(0.1, min(0.8, coast_fraction));  % Limita tra 10% e 80%

% Inizializzazione output
q = zeros(7, 1);
qd = zeros(7, 1);
qdd = zeros(7, 1);

% Tempo normalizzato [0, 1]
tau = max(0, min(1, t / T_final));

% Calcolo tempi delle fasi
tau_coast = coast_fraction;  % Frazione di tempo in coast
tau_accel = (1 - tau_coast) / 2;  % Frazione di tempo in accelerazione
tau_decel = tau_accel;  % Frazione di tempo in decelerazione (simmetrico)

% Tempi di transizione
tau1 = tau_accel;  % Fine accelerazione, inizio coast
tau2 = tau_accel + tau_coast;  % Fine coast, inizio decelerazione

% Tempi assoluti
T_accel = tau_accel * T_final;
T_coast = tau_coast * T_final;
T_decel = tau_decel * T_final;

% Per ogni joint, calcola traiettoria bang-coast-bang indipendente
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
    
    % Calcolo parametri bang-coast-bang
    % Vincoli:
    % 1. Posizione finale: Delta_q = 0.5*a*T_accel^2 + a*T_accel*T_coast + 0.5*a*T_accel^2
    %    Semplificando: Delta_q = a*T_accel*(T_accel + T_coast)
    % 2. Velocità costante durante coast: v_const = a*T_accel
    
    % Accelerazione necessaria
    a_max = Delta_q / (T_accel * (T_accel + T_coast));
    
    % Velocità costante durante coast
    v_const = a_max * T_accel;
    
    if tau <= tau1
        % FASE 1: Accelerazione (0 <= tau <= tau1)
        t_local = tau * T_final;
        
        q(i) = q0(i) + 0.5 * a_max * t_local^2;
        qd(i) = a_max * t_local;
        qdd(i) = a_max;
        
    elseif tau <= tau2
        % FASE 2: Coast - velocità costante (tau1 < tau <= tau2)
        t_local = tau * T_final;
        t_coast_local = t_local - T_accel;
        
        % Posizione alla fine dell'accelerazione
        q_end_accel = q0(i) + 0.5 * a_max * T_accel^2;
        
        % Posizione durante coast
        q(i) = q_end_accel + v_const * t_coast_local;
        qd(i) = v_const;
        qdd(i) = 0;
        
    else
        % FASE 3: Decelerazione (tau2 < tau <= 1)
        t_local = tau * T_final;
        t_decel_local = t_local - T_accel - T_coast;
        
        % Posizione alla fine del coast
        q_end_coast = q0(i) + 0.5 * a_max * T_accel^2 + v_const * T_coast;
        
        % Posizione durante decelerazione
        q(i) = q_end_coast + v_const * t_decel_local - 0.5 * a_max * t_decel_local^2;
        qd(i) = v_const - a_max * t_decel_local;
        qdd(i) = -a_max;
    end
end

% Verifica consistenza: alla fine dovremmo avere velocità zero
if abs(tau - 1.0) < 1e-6
    qd = zeros(7, 1);  % Assicura velocità zero alla fine
end

end