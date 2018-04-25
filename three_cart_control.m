%Three Cart Controller
%Author: Alex McHugh
%Created: 18/04/18
%Last Edited: 22/04/18

clear
close all
clc

%% Parameters

%Masses [kg]
m1 = 1.608;

m2a = 0.75;
m2b = 1.25;
m2 = m2a; %||m2b

m3a = 0.75; 
m3b = 1.25;
m3 = m3a; %||m3b

%Damping %[Ns/m]
c1 = 0; 
c2 = 3.68;
c3 = 3.68;

%Springs %[N/m]
ka = 175;
kb = 400;
kc = 800;
k = ka; %|| kb || kc

%Input Force
alpha = 12.45;%fiddle factor
km = 0.00176; %back emf constant
kg = 3.71; %gear ratio
R = 1.4; %armature resistance [ohms]
r = 0.0184; %pinion radius [m]

beta = alpha * (km*kg)/(R*r);
gamma = (km^2*kg^2)/(R*r^2);

%% System
M = diag([m1 m2 m3]);
C = diag([c1+gamma c2 c3]);
K = [ k -k    0;
     -k  2*k -k;
      0 -k    k];
f = [beta; 0; 0];

  
A = [ zeros(3)  eye(3);
     -inv(M)*K -inv(M)*C]; %Plant Matrix
B1 = [zeros(3,1);
      inv(M)*f]; %Input
C1 = [eye(3) zeros(3)]; %Cart Positions
C2 = [0 0 1 0 0 0]; %Cart 3 Position

%% Control and Tracking
P = [ -2 -20 -50 -40 -4 -10 ]; %desired poles
K = place(A, B1, P); %control gains

ACL = A - B1*K; %closed-loop plant
N = -(C2*ACL^-1*B1)^-1; %tracking gain
B1hat = @(r) B1*N*r; %tracking input matrix


%% Simulation
dt = 0.001; %sampling period
tf = 10; %time duration
%t = 0:dt:tf; %time vector
T = 2; %step train period

steptrain = gensig('square', T, tf, dt) - 1/2;

%250mm Step Input
r250 = 0.250;
sys250 = ss(ACL, B1hat(r250), C2, 0);
[y, t, x] = step(sys250);
[V, dV] = controlValue(x, K, N, r250);

figure
subplot(2,1,1)
stepplot(sys250);
line(xlim, [r250 r250], 'Color', 'k')
title('250mm Step Input')

subplot(2,1,2)
plot(t, V, 'r')

S = stepinfo(sys250);
checkResponse(V, dV, y, r250);

% %500mm Step Input
% r500 = 0.500;
% sys500 = ss(ACL, B1hat(r500), C2, 0);
% figure
% step(sys500)
% line(xlim, [r500 r500], 'Color', 'k')
% title('500mm Step Input')

% % 250mm Step Train Input
% figure
% lsim(sys250, steptrain, t)
% title('250mm Step Train Input')
% 
% %500mm Step Train Input
% figure
% lsim(sys500, steptrain, t)
% title('500mm Step Train Input')

function [err, VPass, dVPass, setPass] = checkResponse(V, dV, y, r)
    %Apparatus Limits
    VLim = 12; %+/- V
    dVLim = 30; %+/- V/s
    setTol = 10e-3; %+/- m
    
    %Tests
    err = r - y(end);
    VPass = isempty(find(abs(V) > VLim, 1));
    dVPass = isempty(find(abs(dV) > dVLim, 1));
    setPass = abs(y(end) - r) < setTol;
    
    %Printed Results
    resp = ["BAD","OK"];
    fprintf("Error of %f [m] (%.1f%%)\n", err, err/r*100);
    fprintf("Voltage %s. max = %.2f / %d [V]\n", ...
            resp(VPass+1), max(abs(V)), VLim);
    fprintf("Slew Rate %s. max = %.2f / %d [V/s]\n", ...
            resp(dVPass+1), max(abs(dV)), dVLim);
    fprintf("Settling %s. final = %.3f / %d [m]\n", ...
            resp(setPass+1), y(end), r);
end
