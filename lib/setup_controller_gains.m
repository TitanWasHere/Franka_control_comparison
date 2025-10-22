function gains = setup_controller_gains(Kp, Kd, Lambda)
    if nargin < 1 || isempty(Kp)
        Kp = diag([40, 40, 40, 40, 30, 30, 30]);
        %Kp = diag([100, 100, 100, 100, 100, 100, 100]);
    end
    
    if nargin < 2 || isempty(Kd)
        Kd = diag([0.5, 1, 1.5, 0.01, 0.02, 0.06, 0.02]);
    end
    
    if nargin < 3 || isempty(Lambda)
        %Lambda = diag([15, 15, 15, 15, 12, 12, 12]);
        %Lambda = Kd \ Kp;
        Lambda = diag([100, 100, 100, 130, 200, 150, 100]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Lambda = Lambda;
end
