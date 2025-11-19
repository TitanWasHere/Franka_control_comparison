function gains = PBC_gains(isOptimized)
    if nargin < 1, isOptimized = false; end

    % default gains
    Kp = diag([100, 100, 100, 100, 50, 50, 50]);
    Kd = diag([20, 20, 20, 20, 10, 10, 10]);

    % optimized gains
    if isOptimized == true
        Kp = diag([120, 250, 120, 300, 250, 200, 60]);
        Kd = diag([ 50,  60,  50,  20, 80, 20, 30]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Lambda = Kd \ Kp; 
end
