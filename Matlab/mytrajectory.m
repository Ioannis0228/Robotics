function [q, qd, qdd] = mytrajectory(q0, qf, t)
% mytrajectory  5th-degree polynomial trajectory 
%
% [q, qd, qdd] = mytrajectory(q0, qf, t)
%
% Inputs:
%   q0 : initial angle (scalar)
%   qf : final angle (scalar)
%   t  : 1xN or Nx1 time vector 
%
% Outputs:
%   q   : 1xN or Nx1 position trajectory
%   qd  : velocity trajectory
%   qdd : acceleration trajectory

    % Normalize time
    T = t(end);     % Assume start is at t(0) and ends at t(end)
    tau = t / T;    % Normalize time to [0, 1]

    % Coefficients of quintic trajectory 
    delta = qf - q0;
    q   = q0 + delta * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    qd  = delta * (30*tau.^2 - 60*tau.^3 + 30*tau.^4) ./ T;
    qdd = delta * (60*tau - 180*tau.^2 + 120*tau.^3) ./ T^2;
end
