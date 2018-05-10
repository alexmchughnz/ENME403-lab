%Inverted Pendulum Controller
%Author: Alex McHugh
%Created: 25/04/18
%Last Edited: 25/04/18

clear
close all
clc

%% Actual Data
ip1 = loadPendulumData('adm101s1');
ip2 = loadPendulumData('adm101s2');
ip = ip2;

tf1 = 1.5;
tf2 = ip.t(end);
tf = tf2;


figure(1)
subplot(2,1,1)
hold on
plot(ip.t, ip.x, 'b')
xlim([0 tf])

subplot(2,1,2)
hold on
plot(ip.t, ip.theta, 'g')
xlim([0 tf])

figure(2)
hold on
plot(ip.t, ip.V, 'r')
xlim([0 tf])


%% Parameters
%Apparatus Limits
VLim = 10; %+/- V
dVLim = 30; %+/- V/s
stLim = 7; %s
rtLim = 3; %s
osLim = 10; %deg
errTol = 0.02 * 0.1; %+/- m

%Physical Properties & Constants
Mp = 0.215; %pendulum mass [kg]
Mc = 1.608; %cart mass [kg]
Lp = 0.314; %pendulum half-length[m]
I0 = 7.06e-3; %inertia [kg m^2] 
R = 0.16; %motor resistance [ohms]
r = 0.0184; %pinion radius [m]
kg = 3.71; %gear ratio
km = 1.68e-2; %back EMF constant [V / (rad / sec)]
C = 0; %damping
g = 9.81; %acceleration due to gravity [m/s^2]

%% System
alpha = (Mc + Mp)*I0 + Mc*Mp*Lp^2;
beta = I0 + Mp*Lp^2;
gamma = C*R*r^2 - km^2*kg^2;

A = [zeros(2) eye(2);
    0 -Mp^2*Lp^2*g/alpha (beta*gamma)/(alpha*R*r^2) 0;
    0 (Mc+Mp)*Mp*Lp*g/alpha -Mp*Lp*gamma/(alpha*R*r^2) 0];
B1 = [0;
     0;
     (beta*km*kg)/(alpha*R*r);
     (-Mp*Lp*km*kg)/(alpha*R*r)];
C1 = [1 0 0 0;
      0 1 0 0]; %x and theta
C2 = [1 0 0 0]; %x

%% Control and Tracking
lqrMode = true;
realMode = true;

if(realMode)
    %Actual Gains/Poles
    K = -abs(ip.K);
    P = eig(A-B1*K);
elseif (lqrMode)
    %LQR
    R = 1.0;
    Q = diag([5 1 0.001 1]);
    [K, ~, P] = lqr(A, B1, Q, R);
else
    %Pole Placement
    P = [ -10; -20; -30; -4.8 ]; %desired poles
    K = place(A, B1, P); %control gains
end

%System
ACL = A - B1*K; %closed-loop plant
N = -(C2*ACL^-1*B1)^-1; %tracking gain
B1hat = @(r) B1*N*r; %tracking input matrix

%% Simulation
%Parameters
r = 0.1; %[m]
 
%Simulation
if realMode
    x0 = [ip.x(1) ip.theta(1) ip.xdot(1) ip.thetadot(1)];
else
    x0 = zeros(1,4);
end

sys = ss(ACL, B1hat(r), C1, 0);
[y, t, x] = lsim(sys, ones(1, length(ip.t)), ip.t, x0);
xc = y(:,1); %cart displacement
theta = y(:,2);
[V, dV] = controlValue(x, K, N, r);

%Results
figure(1)
subplot(2,1,1)
plot(t, xc, '--')
line(xlim, [r r], 'Color', 'k')
if (lqrMode)
    title([num2str(r * 1e3), 'mm step with LQR'])
else
    title([num2str(r * 1e3), 'mm step with poles = [', num2str(sort(P')), ']'])
end
ylabel('x [m]')
legend('Actual', 'Simulated')

subplot(2,1,2)
plot(t, theta, '--')
ylabel('\theta [rad]')
line(xlim, [0 0], 'Color', 'k')

figure(2)
plot(t, V, '--')
line(xlim, [0 0], 'Color', 'k')
ylabel('V [V]')
xlabel('time [s]')

%Printed Info
Sx = stepinfo(xc, t, r);
St = stepinfo(theta, t);
checkResponse(V, dV, xc, r, Sx.SettlingTime, VLim, dVLim, errTol);
disp('x')
stepCheck(r, Sx, stLim, rtLim, 0, errTol, [1 1 1 0 1])
disp('theta')
stepCheck(0, St, stLim, 0, osLim, errTol, [1 1 0 1 1])


