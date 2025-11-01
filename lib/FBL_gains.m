function gains = FBL_gains(isOptimized)
    if nargin < 1, isOptimized = false; end
        
    % Default gains
    Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    Kd = diag([20, 20, 20, 20, 10, 10, 10]);

    % founded those values by searching for better performance
    if isOptimized == true
        Kp = diag([200, 200, 200, 200, 100, 100, 100]);
        Kd = diag([30, 30, 30, 30, 20, 20, 20]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
end
