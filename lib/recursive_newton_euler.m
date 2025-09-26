function [M, c, g] = recursive_newton_euler(q, qd, qdd, params_vec)
%RECURSIVE_NEWTON_EULER Implementazione ricorsiva Newton-Euler per dinamiche robot
%
% Input:
%   q - configurazioni giunti [7x1]
%   qd - velocità giunti [7x1] 
%   qdd - accelerazioni giunti [7x1]
%   params_vec - parametri robot (incerti o veri)
%
% Output:
%   M - matrice inerzia [7x7]
%   c - vettore Coriolis+centrifuga [7x1]
%   g - vettore gravità [7x1]

% Estrai parametri robot
[DH, masses, r_com, I_com] = get_robot_parameters(params_vec);

n = 7; % numero giunti

% Gravità
g0 = [0; 0; -9.81]; % m/s^2

%% CALCOLO MATRICE INERZIA M
M = zeros(n, n);

for j = 1:n
    % Unit vector per colonna j
    qdd_unit = zeros(n, 1);
    qdd_unit(j) = 1;
    
    % Forward dynamics con accelerazione unitaria in giunto j
    tau_j = newton_euler_forward(q, qd, qdd_unit, DH, masses, r_com, I_com, g0);
    
    % Colonna j della matrice inerzia
    M(:, j) = tau_j;
end

%% CALCOLO VETTORE CORIOLIS c
% c = tau(q,qd,0) - g(q)
tau_coriolis = newton_euler_forward(q, qd, zeros(n,1), DH, masses, r_com, I_com, [0;0;0]);
tau_gravity = newton_euler_forward(q, zeros(n,1), zeros(n,1), DH, masses, r_com, I_com, g0);

c = tau_coriolis - tau_gravity;

%% CALCOLO VETTORE GRAVITÀ g
g = tau_gravity;

end

function tau = newton_euler_forward(q, qd, qdd, DH, masses, r_com, I_com, g0)
%NEWTON_EULER_FORWARD Algoritmo Newton-Euler forward ricorsivo

n = length(q);
tau = zeros(n, 1);

% Inizializzazione
w = zeros(3, n+1);      % velocità angolari
wd = zeros(3, n+1);     % accelerazioni angolari  
vd = zeros(3, n+1);     % accelerazioni lineari
vd(:, 1) = -g0;         % accelerazione base (gravità)

% Transformation matrices
T = zeros(4, 4, n+1);
T(:,:,1) = eye(4);      % Base frame

%% FORWARD RECURSION
for i = 1:n
    % DH parameters per link i
    a = DH(i, 1);
    alpha = DH(i, 2);
    d = DH(i, 3);
    theta = q(i) + DH(i, 4);
    
    % Transformation matrix i-1 to i
    T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
           0,           sin(alpha),             cos(alpha),            d;
           0,           0,                      0,                     1];
    
    T(:,:,i+1) = T(:,:,i) * T_i;
    
    % Rotation matrix da frame i-1 a frame i
    R = T_i(1:3, 1:3);
    
    % Unit vectors
    z_prev = [0; 0; 1]; % z axis del frame precedente
    
    % Velocità angolare
    w(:, i+1) = R' * w(:, i) + qd(i) * [0; 0; 1];
    
    % Accelerazione angolare  
    wd(:, i+1) = R' * wd(:, i) + cross(R' * w(:, i), qd(i) * [0; 0; 1]) + qdd(i) * [0; 0; 1];
    
    % Accelerazione lineare dell'origine
    p_i = T_i(1:3, 4); % Posizione origine frame i nel frame i-1
    vd(:, i+1) = R' * (cross(wd(:, i), p_i) + cross(w(:, i), cross(w(:, i), p_i)) + vd(:, i));
end

%% BACKWARD RECURSION  
f = zeros(3, n+1);      % forze
mu = zeros(3, n+1);     % momenti

for i = n:-1:1
    % Accelerazione centro di massa
    vd_com = vd(:, i+1) + cross(wd(:, i+1), r_com(:, i)) + cross(w(:, i+1), cross(w(:, i+1), r_com(:, i)));
    
    % Forza sul centro di massa
    F = masses(i) * vd_com;
    
    % Momento sul centro di massa
    N = I_com(:,:,i) * wd(:, i+1) + cross(w(:, i+1), I_com(:,:,i) * w(:, i+1));
    
    % Trasformazione al frame successivo
    if i == n
        f(:, i+1) = zeros(3, 1);
        mu(:, i+1) = zeros(3, 1);
    end
    
    % Rotation matrix da frame i+1 a frame i
    if i < n
        T_next = zeros(4,4);
        a = DH(i+1, 1);
        alpha = DH(i+1, 2);
        d = DH(i+1, 3);
        theta = q(i+1) + DH(i+1, 4);
        
        T_next = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                  sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                  0,           sin(alpha),             cos(alpha),            d;
                  0,           0,                      0,                     1];
        
        R_next = T_next(1:3, 1:3);
        p_next = T_next(1:3, 4);
    else
        R_next = eye(3);
        p_next = zeros(3, 1);
    end
    
    % Forze e momenti totali
    f(:, i) = R_next * f(:, i+1) + F;
    mu(:, i) = R_next * mu(:, i+1) + cross(r_com(:, i), F) + cross(p_next, R_next * f(:, i+1)) + N;
    
    % Coppia giunto
    tau(i) = mu(3, i); % Proiezione su asse z del giunto
end

end