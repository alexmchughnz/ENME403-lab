%Three Cart Controller
%Author: Alex McHugh
%Created: 18/04/18
%Last Edited: 18/04/18

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

u = @(V, vd1) beta*V - gamma*vd1; %input voltage


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
 
%% Simulation
sys = ss(A, B1, C2, 0);

dt = 0.001; %sampling period
tf = 5; %time duration
t = 0:dt:tf; %time vector

T = 0.5; %step train period

%250mm Step Input
u250 = 0.250;
figure
step(sys, stepDataOptions('StepAmplitude', u250))
line([0 tf], [u250 u250])

%500mm Step Input
figure
step(sys, stepDataOptions('StepAmplitude', 0.500))

%250mm Step Train Input
u250T = 0.250*gensig('square', T, tf, dt) - 0.250/2;
figure
lsim(sys, u250T, t)

%500mm Step Train Input
u500T = 0.500*gensig('square', T, tf, dt) - 0.500/2;
figure
lsim(sys, u500T, t)
title('500mm Step Input')

