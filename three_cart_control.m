%Three Cart Controller
%Author: Alex McHugh
%Created: 18/04/18
%Last Edited: 22/04/18

clear
close all
clc


%% Actual Data
tc1 = loadCartData('adm181s1');
tc2 = loadCartData('adm181s2');
tc3 = loadCartData('adm181s3');
tcArray = {tc1 tc2 tc3};

tcn = 2; %select gain setup
tc = tcArray{tcn};

Tarray = [13.262 11.072 3.3980];
T = Tarray(tcn);
rArray = [0.500 0.500 0.250];
r = rArray(tcn);

if tcn == 1 || tcn == 2
    i0 = find(tc.r > 0, 1);
    tc.t = tc.t(i0:end) - tc.t(i0);
    tc.r = tc.r(i0:end);
    tc.x3 = tc.x3(i0:end);
    tc.V = tc.V(i0:end);
    tf = 10; %display limit
    
elseif tcn == 3
    i0 = find(tc.r(17002:end) > 0, 1) + 17002;
    tc.t = tc.t(i0:end) - tc.t(i0);
    tc.r = tc.r(i0:end);
    tc.x3 = tc.x3(i0:end);
    tc.V = tc.V(i0:end);
    tf = 2*T; %display limit
end




%Plot real data
figure(1)
hold on
plot(tc.t, tc.r, 'k')
plot(tc.t, tc.x3, 'b')
xlim([0 tf])

figure(2)
hold on
plot(tc.t, tc.V, 'r')
xlim([0 tf])

%% Parameters
%Apparatus Limits
VLim = 12; %+/- V
dVLim = 30; %+/- V/s
setTol = 10e-3; %+/- m

%Masses [kg]
m1 = 1.608;

m2a = 0.75;
m2b = 1.25;
m2 = m2b; %||m2b

m3a = 0.75; 
m3b = 1.25;
m3 = m3b; %||m3b

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
Ra = 1.4; %armature resistance [ohms]
rp = 0.0184; %pinion radius [m]

beta = alpha * (km*kg)/(Ra*rp);
gamma = (km^2*kg^2)/(Ra*rp^2);

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
lqrMode = false;
realMode = true;

if(realMode)
    %Actual Gains/Poles
    K = tc.K;
    P = eig(A-B1*K);
elseif (lqrMode)
    %LQR
    R = 1.0;
    Q = diag([0.01 0.01 200 0.01 0.01 10]);
    [K, ~, P] = lqr(A, B1, Q, R);
else
    %Pole Placement
    P = [-12+5i -12-5i -8+4i -8-4i -7 -6]; %desired poles
    K = place(A, B1, P); %control gains
end

%System
ACL = A - B1*K; %closed-loop plant
N = -(C2*ACL^-1*B1)^-1; %tracking gain
B1hat = @(r) B1*N*r; %tracking input matrix


%% Simulation
%Parameters
r250 = 0.250; %250mm step input
r500 = 0.500; %500mm step input

%Simulation Settings
trainOn = true;

if realMode
    x0 = [tc.x1(1) tc.x2(1) tc.x3(1) tc.x1dot(1) tc.x1dot(2) tc.x1dot(3)];
else
    r = r250; %tracking input
    T = 3.3980; %step train period
    x0 = zeros(1,6);
end
%Simulation
sys = ss(ACL, B1hat(r), C2, 0);

if (trainOn)
    [steptrain, t] = gensig('square', T);
    steptrain = 1/2 - steptrain;
    [y, t, x] = lsim(sys, steptrain, t, x0);
else
    [y, t, x] = step(sys);
end

[V, dV] = controlValue(x, K, N, r);

%Results
figure(1)
plot(t, y, '-.')
hold on
ylabel('cart 3 position [m]')
xlabel('time [s]')

if (trainOn)
    %plot(t, steptrain*r, 'k')
else
    line(xlim, [r r], 'Color', 'k')
end

trainStr = {'', ['train (T = ', num2str(T), 's) ']};
if (lqrMode)
    title([num2str(r * 1e3), 'mm step ', trainStr(trainOn+1), 'with LQR'])
else
	title([num2str(r * 1e3), 'mm step ', trainStr(trainOn+1)])
end

figure(2)
plot(t, V, '-.')
line(xlim, [0 0], 'Color', 'k')
ylabel('motor voltage [V]')
xlabel('time [s]')

if (trainOn) 
    target = r/2;
    
else
    target = r;
end

S = lsiminfo(y, t, target);
checkResponse(V, dV, y, target, S.SettlingTime, VLim, dVLim, setTol);
fprintf('Peak at: %d\n', t(find(y>target-1e-3,1)))
