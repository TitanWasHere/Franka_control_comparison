function gains = PBC_gains(isOptimized)

    if nargin < 1 || isempty(isOptimized)
        isOptimized = true;
    end

    %Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    Kd = diag([20, 20, 20, 20, 10, 10, 10]);
    Lambda = diag([100, 100, 100, 50,50,50,50]);

    if isOptimized == true
        %Kp = diag([40, 40, 40, 40, 30, 30, 30]);
        Kd = diag([0.5, 1, 1.5, 0.01, 0.02, 0.06, 0.02]);
        Lambda = diag([100, 100, 100, 130, 200, 150, 100]);
    end
    
    %gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Lambda = Lambda;
end
