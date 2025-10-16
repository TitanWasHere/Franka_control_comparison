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