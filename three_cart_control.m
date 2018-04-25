%Three Cart Controller
%Author: Alex McHugh
%Created: 18/04/18
%Last Edited: 22/04/18

clear
close all
clc

%% Parameters
%Apparatus Limits
VLim = 12; %+/- V
dVLim = 30; %+/- V/s
setTol = 10e-3; %+/- m

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
%Parameters
r250 = 0.250; %250mm step input
r500 = 0.500; %500mm step input

%Simulation Settings
r = r250; %tracking input
trainOn = true;
T = 1; %step train period

%Simulation
sys = ss(ACL, B1hat(r), C2, 0);

if (trainOn)
    [steptrain, t] = gensig('square', T);
    steptrain = steptrain - 1/2;
    [y, t, x] = lsim(sys, steptrain, t);
else
    [y, t, x] = step(sys);
end

[V, dV] = controlValue(x, K, N, r);

%Results
figure
subplot(2,1,1)
plot(t, y, 'b')
hold on

if (trainOn)
    plot(t, steptrain*r, 'k')
else
    line(xlim, [r r], 'Color', 'k')
end

trainStr = {'', ['train (T = ', num2str(T), 's) ']};
title([num2str(r * 1e3), 'mm step ', trainStr{trainOn+1} ...
    'with poles = [', num2str(sort(P)), ']'])
ylabel('input amplitude [m]')

subplot(2,1,2)
plot(t, V, 'r')
ylabel('motor voltage [V]')
xlabel('time [s]')

if (trainOn) 
    target = r/2;
else
    target = r;
end
checkResponse(V, dV, y, target, VLim, dVLim, setTol);

% % 250mm Step Train Input
% figure
% lsim(sys250, steptrain, t)
% title('250mm Step Train Input')
% 
% %500mm Step Train Input
% figure
% lsim(sys500, steptrain, t)
% title('500mm Step Train Input')


