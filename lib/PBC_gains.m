function gains = PBC_gains(isOptimized)
    if nargin < 1, isOptimized = false; end

    % default gains
    Kp = diag([40, 40, 40, 40, 30, 30, 30]);
    Kd = diag([30, 30, 30, 30, 20, 20, 20]);

    % optimized gains
    if isOptimized == true
        Kp = diag([120, 120, 120, 120, 60, 60, 60]);
        Kd = diag([ 50,  50,  50,  50, 30, 30, 30]);
    end
    
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Lambda = Kd \ Kp; 
end
