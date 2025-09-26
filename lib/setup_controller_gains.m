function gains = setup_controller_gains(Kp, Kd, Lambda)
    if nargin < 1 || isempty(Kp)
        Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    end
    
    if nargin < 2 || isempty(Kd)
        Kd = diag([20, 20, 20, 20, 10, 10, 10]);
    end
    
    if nargin < 3 || isempty(Lambda)
        Lambda = diag([10, 10, 10, 10, 6, 6, 6]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Lambda = Lambda;
end