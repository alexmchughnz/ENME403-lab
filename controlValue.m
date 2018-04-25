function [ u, du ] = controlValue(x, K, N, r)
%controlValue Computes control effort from state values over time.
%Designed for a tracking system where u = -Kx + Nr.

%OUTPUTS:
% u - Control input.
% du - (approx.) Rate of change. du(ii) = u(ii+1) - u(ii)

u = -K*x' + N*r;

du = u(1:end-1) - u(2:end);


end

