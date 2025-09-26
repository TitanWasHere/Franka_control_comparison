function [M, c, g, tau_f] = fast_dynamics(q, qd, params_vec)
    %FAST_DYNAMICS Calcolo dinamiche robot usando algoritmi numerici O(n)
    %
    % Input:
    %   q - configurazione giunto [7x1]
    %   qd - velocità giunto [7x1] 
    %   params - parametri robot 
    %
    % Output:
    %   M - matrice inerzia [7x7]
    %   c - vettore Coriolis [7x1]
    %   g - vettore gravità [7x1]
    %   tau_f - coppia attrito [7x1]
    
    % Usa algoritmo Newton-Euler ricorsivo con parametri incerti
    [M, c, g] = recursive_newton_euler(q, qd, zeros(7,1), params_vec);
    
    % Calcola attrito
    tau_f = compute_friction(qd, params_vec);
end

