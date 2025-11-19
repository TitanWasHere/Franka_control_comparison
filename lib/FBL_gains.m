function gains = FBL_gains(isOptimized)
    if nargin < 1, isOptimized = false; end
        
    % Default gains
    Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    Kd = diag([20, 20, 20, 20, 10, 10, 10]);

    % founded those values by searching for better performance
    if isOptimized == true
        Kp = diag([200, 300, 400, 750, 600, 1200, 700]);
        Kd = diag([30, 50, 50, 80, 200, 600, 400]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
end
