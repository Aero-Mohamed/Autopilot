clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia
plane = AirPlane("NT-33A_4.xlsx");
steps = (plane.timeSpan(2) - plane.timeSpan(1))/plane.dt;
Result = NaN(12, steps);
Result(:,1) = plane.ICs;
time_V = linspace(0, plane.timeSpan(2), steps+1);

%% Solving
%profile on;
dForces = [0 ; 0; 0];
dMoments = [0 ; 0; 0];

for i =1:steps
    Result(:, i+1) = plane.rigidBodySolver.nextStep( ...
        Result(:, i),(plane.initialGravity + dForces), dMoments ...
    );    

    [dF, dM] = plane.airFrame1(Result(:, i+1), ...
        (plane.initialGravity + dForces), dMoments, plane.dControl ...
    );
    

    dForces = vpa(dF');
    dMoments = vpa(dM');
    
end
%profile viewer

%% Rearranging Results
u = Result(1,:); v = Result(2,:); w = Result(3,:);
p = Result(4,:); q = Result(5,:); r = Result(6,:);
phi = Result(7,:); theta = Result(8,:); psi = Result(9,:);
x = Result(10,:); y = Result(11,:); z = Result(12,:);

beta_deg=asin(v/plane.Vt0)*180/pi;
alpha_deg=atan(w./u)*180/pi;
p_deg=p*180/pi;
q_deg=q*180/pi;
r_deg=r*180/pi;
phi_deg=phi*180/pi;
theta_deg=theta*180/pi;
psi_deg=psi*180/pi;

%% Longitudenal Full Linear Model

% Two Inputs - Four Output Each
[A_long, B_long, C_long, D_long] = plane.fullLinearModel();
LongSS = ss(A_long, B_long, C_long, D_long);

%%% Due to delta_elevetor or delta_thrust
opt = stepDataOptions;
opt.StepAmplitude = plane.dControl(3:4); % dE, dTh

[res, ~, ~] = step(LongSS, time_V, opt);
long_res_dE  = res(:,:,1);
long_res_dTh = res(:,:,2);

%% PHUGOID MODE (LONG PERIOD MODE)

[A_phug, B_phug, C_phug, D_phug] = plane.longPeriodModel();
PHUG_SS = ss(A_phug,B_phug,C_phug,D_phug);

[res, ~, ~] = step(PHUG_SS, time_V, opt);
phug_res_dE = res(:, :, 1);
phug_res_dTh = res(:, :, 2);

%% theta response Full Linear - Approximate - Non Linear 
figure
if(plane.dControl(3) ~= 0)
    % dE input
    theta_ = (long_res_dE(:, 4) + plane.theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model  
    hold on
    theta_ = (phug_res_dE(:, 2) + plane.theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Long Period
elseif(plane.dControl(4) ~= 0)
    % dTh input
    theta_ = (long_res_dTh(:, 4) + plane.theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model
    hold on
    theta_ = (phug_res_dTh(:, 2) + plane.theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Long Period
end

hold on
plot(time_V, theta_deg, '-', 'DisplayName', '\Theta (Non-Linear)');                  % Non-Linear Model
title('theta (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on
