clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia

[Mass, g, I, invI, timeSpan, dt, ICs, ICs_dot0, Vt0, ...
    dControl, SD_Long, SD_Lat,SD_Lat_dash, initialGravity] = Input("NT-33A_4.xlsx");
steps = (timeSpan(2) - timeSpan(1))/dt;
Result = NaN(12, steps);
Result(:,1) = ICs;
time_V = linspace(0, timeSpan(2), steps+1);

%% Solving
%profile on;
dForces = [0 ; 0; 0];
dMoments = [0 ; 0; 0];

for i =1:steps
    
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
v0 = ICs(2);
w0 = ICs(3);
q0 = ICs(5);
theta0 = ICs(8);

%%  Longitudinal Full Linear Model Step Response
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

%%% Due to delta_elevetor or delta_thrust
dControl_long = dControl(3:4); % dE, dTh
opt = stepDataOptions;
opt.StepAmplitude = dControl_long;
[res, ~, ~] = step(LongSS, time_V, opt);
long_res_dE  =res(:,:,1);
long_res_dTh =res(:,:,2);

%% PHUGOID MODE (LONG PERIOD MODE)

A_phug=[XU -g
    -ZU/(u0+ZQ) 0];
B_phug=[XDE XDTH
    -ZDE/(ZQ+u0) -ZDTH/(ZQ+u0)];
C_phug=eye(2);D_phug=zeros(2,2);

PHUG_SS = ss(A_phug,B_phug,C_phug,D_phug);
[res, ~, ~] = step(PHUG_SS, time_V, opt);
phug_res_dE = res(:, :, 1);
phug_res_dTh = res(:, :, 2);

%% theta response Full Linear - Approximate - Non Linear 
figure
if(dControl_long(1) ~= 0)
    % dE input
    theta_ = (long_res_dE(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model  
    hold on
    theta_ = (phug_res_dE(:, 2) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Long Period
elseif(dControl_long(2) ~= 0)
    % dTh input
    theta_ = (long_res_dTh(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model
    hold on
    theta_ = (phug_res_dTh(:, 2) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Long Period
end

hold on
plot(time_V, theta_deg, '-', 'DisplayName', '\Theta (Non-Linear)');                  % Non-Linear Model
title('theta (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on
