clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia

[Mass, g, I, invI, timeSpan, dt, ICs, ICs_dot0, Vt0, ...
    dControl, SD_Long, SD_Lat, initialGravity] = Input("NT-33A_4.xlsx");
steps = (timeSpan(2) - timeSpan(1))/dt;
Result = NaN(12, steps);
Result(:,1) = ICs;
time_V = linspace(0, timeSpan(2), steps+1);

%% Solving
%profile on;
dForces = [0 ; 0; 0];
dMoments = [0 ; 0; 0];

for i =1:steps
    i
    Result(:, i+1) = RBDSolver(Result(:, i), dt, (initialGravity + dForces), dMoments, Mass, I, invI, g);
    
    [dF, dM] = airFrame(SD_Long, SD_Lat, dControl, ICs, ICs_dot0, Result(:, i+1) ,Vt0, ... 
        (initialGravity + dForces), dMoments, Mass, I, invI, g);
    

    dForces = vpa(dF');
    dMoments = vpa(dM');
    
end
%profile viewer

%% Rearranging Results
u = Result(1,:);
v = Result(2,:);
w = Result(3,:);
p = Result(4,:);
q = Result(5,:);
r = Result(6,:);
phi = Result(7,:);
theta = Result(8,:);
psi = Result(9,:);
x = Result(10,:);
y = Result(11,:);
z = Result(12,:);

beta_deg=asin(v/Vt0)*180/pi;
alpha_deg=atan(w./u)*180/pi;
p_deg=p*180/pi;
q_deg=q*180/pi;
r_deg=r*180/pi;
phi_deg=phi*180/pi;
theta_deg=theta*180/pi;
psi_deg=psi*180/pi;

%% Longitudenal Full Linear Model
XU=SD_Long(1);
ZU=SD_Long(2);
MU=SD_Long(3);
XW=SD_Long(4);
ZW=SD_Long(5);
MW=SD_Long(6);
ZWD=SD_Long(7);
ZQ=SD_Long(8);
MWD=SD_Long(9);
MQ=SD_Long(10);
XDE=SD_Long(11);
ZDE=SD_Long(12);
MDE=SD_Long(13);
XDTH=SD_Long(14);
ZDTH=SD_Long(15);
MDTH=SD_Long(16);
u0 = ICs(1);
w0 = ICs(3);
q0 = ICs(5);
theta0 = ICs(8);
A_long=[XU XW -w0 -g*cos(theta0)
    ZU/(1-ZWD) ZW/(1-ZWD) (ZQ+u0)/(1-ZWD) -g*sin(theta0)/(1-ZWD)
    MU+MWD*ZU/(1-ZWD) MW+MWD*ZW/(1-ZWD) MQ+MWD*(ZQ+u0)/(1-ZWD) -MWD*g*sin(theta0)/(1-ZWD)
    0 0 1 0];
B_long=[XDE XDTH
    ZDE/(1-ZWD) ZDTH/(1-ZWD)
    MDE+MWD*ZDE/(1-ZWD) MDTH+MWD*ZDTH/(1-ZWD)
    0 0];
C_long=eye(4); 
D_long=zeros(4,2);

% Two Inputs - Four Output Each
LongSS = ss(A_long, B_long, C_long, D_long);
LongTF = tf(LongSS);
% Due to delta Elevator
U_DE        = LongTF(1,1);
W_DE        = LongTF(2,1);
Q_DE        = LongTF(3,1);
THETA_DE    = LongTF(4,1);
% Due to delta Thrust
U_DTH       = LongTF(1,2);
W_DTH       = LongTF(2,2);
Q_DTH       = LongTF(3,2);
THETA_DTH   = LongTF(4,2);


%% Plotting Longitudinal Full Linear Model Step Response
%%% Due to delta_elevetor or delta_thrust
dControl_long = dControl(3:4); % dE, dTh
opt = stepDataOptions;
opt.StepAmplitude = dControl_long;
[res_dE, ~, res_dTh] = step(LongSS, time_V, opt);

%% u response Full Linear
figure;
if(dControl_long(1) ~= 0)
    plot(time_V, res_dE(:, 1) + u0, '--', 'DisplayName', 'u (Full Linear)');  % Full Linear Model    
elseif(dControl_long(2) ~= 0)
    plot(time_V, res_dTh(:, 1) + u0, '--', 'DisplayName', 'u (Full Linear)');  % Full Linear Model
end

hold on
plot(time_V, u, '-', 'DisplayName', 'u (Non-Linear)');                  % Non-Linear Model
title('u (ft/sec)'); xlabel('t (sec)');
legend('show');

%% w response Full Linear
figure;
if(dControl_long(1) ~= 0)
    plot(time_V, res_dE(:, 2) + w0, '--', 'DisplayName', 'w (Full Linear)');  % Full Linear Model    
elseif(dControl_long(2) ~= 0)
    plot(time_V, res_dTh(:, 2) + w0, '--', 'DisplayName', 'w (Full Linear)');  % Full Linear Model
end

hold on
plot(time_V, w, '-', 'DisplayName', 'w (Non-Linear)');                  % Non-Linear Model
title('w (ft/sec)'); xlabel('t (sec)');
legend('show');

%% q response Full Linear
figure;
if(dControl_long(1) ~= 0)
    q_ = res_dE(:, 3) + q0;
    plot(time_V, q_*180/pi, '--', 'DisplayName', 'q (Full Linear)');  % Full Linear Model    
elseif(dControl_long(2) ~= 0)
    q_ = res_dTh(:, 3) + q0;
    plot(time_V, q_*180/pi, '--', 'DisplayName', 'q (Full Linear)');  % Full Linear Model
end

hold on
plot(time_V, q_deg , '-', 'DisplayName', 'q (Non-Linear)');                  % Non-Linear Model
title('q (deg/sec)'); xlabel('t (sec)');
legend('show');

%% theta response Full Linear
figure;
if(dControl_long(1) ~= 0)
    theta_ = (res_dE(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', 'Theta (Full Linear)');  % Full Linear Model    
elseif(dControl_long(2) ~= 0)
    theta_ = (res_dTh(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', 'Theta (Full Linear)');  % Full Linear Model
end

hold on
plot(time_V, theta_deg, '-', 'DisplayName', 'q (Non-Linear)');                  % Non-Linear Model
title('theta (deg/sec)'); xlabel('t (sec)');
legend('show');

%% RootLocus & Bode Plots Full Linear Model
%% u/de Full Linear
figure
subplot(1, 2, 1);
rlocus(U_DE);
title('LINEAR FULL MODLE root locus (u/\delta_{e}) ');
subplot(1, 2, 2);
bode(U_DE);
title('LINEAR FULL MODLE bode plot (u/\delta_{e}) ');

%% w/de Full Linear
figure
subplot(1, 2, 1);
rlocus(W_DE);
title('LINEAR FULL MODLE root locus (w/\delta_{e}) ');
subplot(1, 2, 2);
bode(W_DE);
title('LINEAR FULL MODLE bode plot (w/\delta_{e}) ');

%% w/de Full Linear
figure
subplot(1, 2, 1);
rlocus(Q_DE);
title('LINEAR FULL MODLE root locus (q/\delta_{e}) ');
subplot(1, 2, 2);
bode(Q_DE);
title('LINEAR FULL MODLE bode plot (q/\delta_{e}) ');

%% theta/de Full Linear
figure
subplot(1, 2, 1);
rlocus(THETA_DE);
title('LINEAR FULL MODLE root locus (\theta/\delta_{e}) ');
subplot(1, 2, 2);
bode(THETA_DE);
title('LINEAR FULL MODLE bode plot (\theta/\delta_{e}) ');

%% u/dTh Full Linear
figure
subplot(1, 2, 1);
rlocus(U_DTH);
title('LINEAR FULL MODLE root locus (u/\delta_{th}) ');
subplot(1, 2, 2);
bode(U_DTH);
title('LINEAR FULL MODLE bode plot (u/\delta_{th}) ');

%% w/dTh Full Linear
figure
subplot(1, 2, 1);
rlocus(W_DTH);
title('LINEAR FULL MODLE root locus (w/\delta_{th}) ');
subplot(1, 2, 2);
bode(W_DTH);
title('LINEAR FULL MODLE bode plot (w/\delta_{th}) ');

%% w/dTh Full Linear
figure
subplot(1, 2, 1);
rlocus(Q_DTH);
title('LINEAR FULL MODLE root locus (q/\delta_{th}) ');
subplot(1, 2, 2);
bode(Q_DTH);
title('LINEAR FULL MODLE bode plot (q/\delta_{th}) ');

%% theta/dTh Full Linear
figure
subplot(1, 2, 1);
rlocus(THETA_DTH);
title('LINEAR FULL MODLE root locus (\theta/\delta_{th}) ');
subplot(1, 2, 2);
bode(THETA_DTH);
title('LINEAR FULL MODLE bode plot (\theta/\delta_{th}) ');

