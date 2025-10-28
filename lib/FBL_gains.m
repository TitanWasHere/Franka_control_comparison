function gains = FBL_gains(isOptimized)

    % Default gains
    Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    Kd = diag([20, 20, 20, 20, 10, 10, 10]);

    % founded those values by searching for better performance
    if isOptimized == true
        Kp = Kp * 2;
        Kd = Kd + 10;
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
end
