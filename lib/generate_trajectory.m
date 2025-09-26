function [q_d, qd_d, qdd_d] = generate_trajectory(t, q0, qf, T_final)
%GENERATE_TRAJECTORY Generates a smooth rest-to-rest joint trajectory.
%   This function computes the desired position, velocity, and acceleration
%   for a given time 't' using a quintic (5th-order) polynomial. This
%   ensures that the trajectory starts and ends at the specified positions
%   (q0, qf) with zero velocity and zero acceleration.
%
%   The trajectory is defined for a time interval [0, T_final].
%
% Inputs:
%   t:        The current time (scalar).
%   q0:       The initial joint position vector (e.g., 7x1).
%   qf:       The final joint position vector (e.g., 7x1).
%   T_final:  The total duration of the trajectory (scalar).
%
% Outputs:
%   q_d:      The desired joint position vector at time t.
%   qd_d:     The desired joint velocity vector at time t.
%   qdd_d:    The desired joint acceleration vector at time t.

% Ensure time is within the valid interval [0, T_final]
if t < 0
    t = 0;
elseif t > T_final
    t = T_final;
end

% The general form of the quintic polynomial is:
% q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
% The coefficients are calculated by solving the system of equations
% defined by the boundary conditions: q(0), q_dot(0), q_ddot(0),
% q(T), q_dot(T), q_ddot(T).

% The function is vectorized: it computes the coefficients for all joints at once.
delta_q = qf - q0;

% Coefficients for the quintic polynomial
a0 = q0;
a1 = zeros(size(q0)); % Initial velocity is zero
a2 = zeros(size(q0)); % Initial acceleration is zero
a3 = (10 / T_final^3) * delta_q;
a4 = -(15 / T_final^4) * delta_q;
a5 = (6 / T_final^5) * delta_q;

% Calculate desired position, velocity, and acceleration at time t
q_d   = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
qd_d  =      a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
qdd_d =           2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;

end