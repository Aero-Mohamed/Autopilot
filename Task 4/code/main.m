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

%% APPROXIMATE 
%% PHUGOID MODE (LONG PERIOD MODE)

A_phug=[XU -g
    -ZU/(u0+ZQ) 0];
B_phug=[XDE XDTH
    -ZDE/(ZQ+u0) -ZDTH/(ZQ+u0)];
C_phug=eye(2);D_phug=zeros(2,2);

PHUG_SS=ss(A_phug,B_phug,C_phug,D_phug);

%% SHORT PERIOD MODE
A_short=[ZW/(1-ZWD) (ZQ+u0)/(1-ZWD)
    (MW+ZW*MWD/(1-ZWD)) (MQ+MWD*(ZQ+u0)/(1-ZWD))];
B_short=[ZDE/(1-ZWD) ZDTH/(1-ZWD)
    MDE+MWD*ZDE/(1-ZWD) MDTH+MWD*ZDTH/(1-ZWD)];
C_short=eye(2);D_short=zeros(2,2);

SHORT_SS=ss(A_short,B_short,C_short,D_short);

%%  Longitudinal Full Linear Model Step Response
%%% Due to delta_elevetor or delta_thrust
dControl_long = dControl(3:4); % dE, dTh
opt = stepDataOptions;
opt.StepAmplitude = dControl_long;
[res, ~, ~] = step(LongSS, time_V, opt);
res_dE  =res(:,:,1);
res_dTh =res(:,:,2);
%% Longitudinal Approximate Models Step Response
%%% Due to delta_elevetor or delta_thrust
dControl_long = dControl(3:4); % dE, dTh
opt = stepDataOptions;
opt.StepAmplitude = dControl_long;
[APPres_PH, ~,~] = step(PHUG_SS, time_V, opt);
[APPres_SH, ~,~] = step(SHORT_SS, time_V, opt);

APPres_dE=zeros (length(time_V),4);
APPres_dE(:,[1,4])=APPres_PH(:,:,1);
APPres_dE(:,[2,3])=APPres_SH(:,:,1);

APPres_dTH=zeros (length(time_V),4);
APPres_dTH(:,[1,4])=APPres_PH(:,:,2);
APPres_dTH(:,[2,3])=APPres_SH(:,:,2);
%% u response Full Linear - Approximate - Non Linear 
figure(1)
if(dControl_long(1) ~= 0)
    plot(time_V, res_dE(:, 1) + u0, '--', 'DisplayName', 'u (Full Linear)');  % Full Linear Model 
    hold on
    plot(time_V, APPres_dE(:, 1) + u0, '--', 'DisplayName', 'u (Long Period Approximation)');  % Approximate (long period Mode)     
elseif(dControl_long(2) ~= 0)
    plot(time_V, res_dTh(:, 1) + u0, '--', 'DisplayName', 'u (Full Linear)');  % Full Linear Model
    hold on
    plot(time_V, APPres_dTH(:, 1) + u0, '--', 'DisplayName', 'u (Long Period Approximation)');  % Approximate (long period Mode)
end

hold on
plot(time_V, u, '-', 'DisplayName', 'u (Non-Linear)');                  % Non-Linear Model
title('u (ft/sec)'); xlabel('t (sec)');
legend('show');
grid on 
%% w response Full Linear - Approximate - Non Linear 
figure(2)
if(dControl_long(1) ~= 0)
    plot(time_V, res_dE(:, 2) + w0, '--', 'DisplayName', 'w (Full Linear)');  % Full Linear Model 
    hold on 
    plot(time_V, APPres_dE(:, 2) + w0, '--', 'DisplayName', 'w (short Period Approximation)');  % Approximate (short period Mode)
elseif(dControl_long(2) ~= 0)
    plot(time_V, res_dTh(:, 2) + w0, '--', 'DisplayName', 'w (Full Linear)');  % Full Linear Model
    hold on
    plot(time_V, APPres_dTH(:, 2) + w0, '--', 'DisplayName', 'w (short Period Approximation)');  % Approximate (short period Mode)
end

hold on
plot(time_V, w, '-', 'DisplayName', 'w (Non-Linear)');                  % Non-Linear Model
title('w (ft/sec)'); xlabel('t (sec)');
legend('show');
grid on
%% q response Full Linear - Approximate - Non Linear 
figure(3)
if(dControl_long(1) ~= 0)
    q_ = res_dE(:, 3) + q0;
    plot(time_V, q_*180/pi, '--', 'DisplayName', 'q (Full Linear)');  % Full Linear Model
    hold on
    q_APP = APPres_dE(:, 3) + q0;
    plot(time_V, q_APP*180/pi, '--', 'DisplayName', 'q (short Period Approximation)');  % Approximate (short period Mode)

elseif(dControl_long(2) ~= 0)
    q_ = res_dTh(:, 3) + q0;
    plot(time_V, q_*180/pi, '--', 'DisplayName', 'q (Full Linear)');  % Full Linear Model
    hold on
    q_APP = APPres_dTH(:, 3) + q0;
    plot(time_V, q_APP*180/pi, '--', 'DisplayName', 'q (short Period Approximation)');  % Approximate (short period Mode)
end

hold on
plot(time_V, q_deg , '-', 'DisplayName', 'q (Non-Linear)');                  % Non-Linear Model
title('q (1/sec^2)'); xlabel('t (sec)');
legend('show');
grid on
%% theta response Full Linear - Approximate - Non Linear 
figure(4)
if(dControl_long(1) ~= 0)
    theta_ = (res_dE(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model  
    hold on
    theta_APP = (APPres_dE(:, 4) + theta0)*180/pi;
    plot(time_V, theta_APP, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Full Linear Model  
elseif(dControl_long(2) ~= 0)
    theta_ = (res_dTh(:, 4) + theta0)*180/pi;
    plot(time_V, theta_, '--', 'DisplayName', '\Theta (Full Linear)');  % Full Linear Model
    hold on
    theta_APP = (APPres_dTH(:, 4) + theta0)*180/pi;
    plot(time_V, theta_APP, '--', 'DisplayName', '\Theta (Long Period Approximation)');  % Full Linear Model      
end

hold on
plot(time_V, theta_deg, '-', 'DisplayName', '\Theta (Non-Linear)');                  % Non-Linear Model
title('theta (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on
%% Lateral Full Linear Model

U0=u0; W0=w0; TH0=theta0; psi0=ICs(9); phi0=ICs(7);
Vto = sqrt(ICs(1)^2 + ICs(2)^2 + ICs(3)^2);    % Vto

% stability derivatives Lateral motion

Yp=0;
Yr=0;
YDa_star=SD_Lat_dash(9); 
YDr_star=SD_Lat_dash(10);
Yb=SD_Lat_dash(2); 
YDa=SD_Lat_dash(9)*Vto; 
YDr=SD_Lat_dash(10)*Vto;

Lbd=SD_Lat_dash(3); 
Lpd=SD_Lat_dash(5);
Lrd=SD_Lat_dash(7);
LDrd=SD_Lat_dash(13);
LDad=SD_Lat_dash(11);

Nbd=SD_Lat_dash(4); 
Npd=SD_Lat_dash(6);
Nrd=SD_Lat_dash(8);
NDrd=SD_Lat_dash(14);
NDad=SD_Lat_dash(12);


A_Lat=[Yb/Vto (Yp+W0)/Vto (Yr-U0)/Vto g*cos(TH0)/Vto 0;...
       Lbd Lpd Lrd 0 0;...
       Nbd Npd Nrd 0 0;...
       0 1 tan(TH0) 0 0;...
       0 0 1/cos(TH0) 0 0];
B_Lat=[YDa YDr;...
       LDad LDrd;...
       NDad NDrd;...
       0 0;0 0];
C_Lat=eye(5); D_Lat=zeros(5,2);


Lateral_SS = ss(A_Lat,B_Lat,C_Lat,D_Lat);
Lateral_TF = tf(Lateral_SS);

B_DA_L = Lateral_TF(1,1);
B_DR_L = Lateral_TF(1,2);

P_DA_L = Lateral_TF(2,1);
P_DR_L = Lateral_TF(2,2);

R_DA_L = Lateral_TF(3,1);
R_DR_L = Lateral_TF(3,2);

PHI_DA_L = Lateral_TF(4,1);
PHI_DR_L = Lateral_TF(4,2);

PSI_DA_L = Lateral_TF(5,1);
PSI_DR_L = Lateral_TF(5,2);


%% 3DOF Spiral Mode Approximation
A_Spiral = [Lpd Lrd 0;...
            Npd Nrd 0;...
            1 tan(TH0) 0];
B_Spiral = [LDad LDrd;NDad NDrd;0 0];
C_Spiral = eye(3); D_Spiral = zeros(3,2);

Spiral_SS = ss(A_Spiral,B_Spiral,C_Spiral,D_Spiral);
Spiral_TF = tf(Spiral_SS);

P_DA_S= Spiral_TF(1, 1);
P_DR_S = Spiral_TF(1, 2);

R_DA_S = Spiral_TF(2, 1);
R_DR_S = Spiral_TF(2, 2);

PHI_DA_S = Spiral_TF(3, 1);
PHI_DR_S = Spiral_TF(3, 2);


%% 2DOF Dutch Mode Approximation
A_Dutch = [Yb/Vto (Yr-U0)/Vto-tan(TH0)*(Yp+W0)/Vto;Nbd Nrd-tan(TH0)*Npd];
B_Dutch = [YDa_star YDr_star;NDad NDrd];
C_Dutch = eye(2); D_Dutch = zeros(2,2);

Dutch_SS = ss(A_Dutch, B_Dutch, C_Dutch, D_Dutch);
Dutch_TF = tf(Dutch_SS);

B_DA_D = Dutch_TF(1, 1);
B_DR_D = Dutch_TF(1, 2);

R_DA_D = Dutch_TF(2, 1);
R_DR_D = Dutch_TF(2, 2);


%% 1DOF Roll Approximation 
A_Roll = Lpd;
B_Roll = LDad;
C_Roll = eye(1); D_Roll = zeros(1, 1);

Roll_SS = ss(A_Roll, B_Roll, C_Roll, D_Roll);
Roll_TF = tf(Roll_SS);

P_DA_R = Roll_TF(1 ,1);

%%  Lateral Full Linear Model Step Response
%% Due to delta_elevetor or delta_thrust
dControl_latr = dControl(1:2); % dA, dR
opt = stepDataOptions;
opt.StepAmplitude = dControl_latr;
[Lat_res, ~, ~] = step(Lateral_SS, time_V, opt);
Lat_res_dA  =Lat_res(:,:,1);
Lat_res_dR =Lat_res(:,:,2);
%% Lateral Approximate Models Step Response

dControl_latr = dControl(1:2); % dA, dR
opt = stepDataOptions;
opt.StepAmplitude = dControl_latr;

[Spir_res, ~,~] = step(Spiral_SS, time_V, opt);
[Dutch_res, ~,~] = step(Dutch_SS, time_V, opt);

dC1= dControl(1); % dA, dR
opt = stepDataOptions;
opt.StepAmplitude = dC1;
[Roll_res, ~,~] = step(Roll_SS, time_V, opt);

%%% Spiral
Spir_res_dA=zeros (length(time_V),5);
Spir_res_dA(:,[2,3,4])=Spir_res(:,:,1);

Spir_res_dR=zeros (length(time_V),5);
Spir_res_dR(:,[2,3,4])=Spir_res(:,:,2);
%%% Dutch
Dutch_res_dA=zeros (length(time_V),5);
Dutch_res_dA(:,[1,3])=Dutch_res(:,:,1);

Dutch_res_dR=zeros (length(time_V),5);
Dutch_res_dR(:,[1,3])=Dutch_res(:,:,2);
%%% Roll
Roll_res_dA=zeros (length(time_V),5);
Roll_res_dA(:,2)=Roll_res; 
%%
figure(5)
beta0=v0/Vt0;
if(dControl_latr(1) ~= 0)
    beta_ = (Lat_res_dA(:, 1) + beta0)*180/pi;
    plot(time_V, beta_, '--', 'DisplayName', '\beta (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    beta_d = (Dutch_res_dA(:, 1) + beta0)*180/pi;
    plot(time_V, beta_d, '--', 'DisplayName', '\beta (dutch)');  % spiral Linear Model  
elseif(dControl_latr(2) ~= 0)
    beta_ = (Lat_res_dR(:, 1) + beta0)*180/pi;
    plot(time_V, beta_, '--', 'DisplayName', '\beta (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    beta_d = (Dutch_res_dR(:, 1) + beta0)*180/pi;
    plot(time_V, beta_d, '--', 'DisplayName', '\beta (dutch)');  % spiral Linear Model  
end

hold on
plot(time_V, beta_deg, '-', 'DisplayName', '\beta (Non-Linear)');                  % Non-Linear Model
title('\beta (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on
%%
figure(6)
beta0=v0/Vt0;
if(dControl_latr(1) ~= 0)
    P_ = Lat_res_dA(:, 2) ;
    plot(time_V, P_, '--', 'DisplayName', 'P (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    P_r = Roll_res_dA(:, 2);
    plot(time_V, P_r, '--', 'DisplayName', 'P (Roll)');  % spiral Linear Model  
    hold on 
    P_s = Spir_res_dA(:, 2);
    plot(time_V, P_s, '--', 'DisplayName', 'P (Spiral)');  % spiral Linear Model 
elseif(dControl_latr(2) ~= 0)
    P_ = Lat_res_dR(:, 2) ;
    plot(time_V, P_, '--', 'DisplayName', 'P (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    P_s = Spir_res_dR(:, 2);
    plot(time_V, P_s, '--', 'DisplayName', 'P (Spiral)');  % spiral Linear Model 
end

hold on
plot(time_V, p, '-', 'DisplayName', 'p (Non-Linear)');                  % Non-Linear Model
title('p (1/sec^2)'); xlabel('t (sec)');
legend('show');
grid on
%%
figure(7)
if(dControl_latr(1) ~= 0)
    R_ = Lat_res_dA(:, 3) ;
    plot(time_V, R_, '--', 'DisplayName', 'R (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    R_d = Dutch_res_dA(:, 3);
    plot(time_V, R_d, '--', 'DisplayName', 'R (dutch)');  % spiral Linear Model  
    hold on 
    R_s = Spir_res_dA(:, 3);
    plot(time_V, R_s, '--', 'DisplayName', 'R (Spiral)');  % spiral Linear Model 
elseif(dControl_latr(2) ~= 0)
    R_ = Lat_res_dR(:, 3) ;
    plot(time_V, R_, '--', 'DisplayName', 'R (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    R_d = Dutch_res_dR(:, 3);
    plot(time_V, R_d, '--', 'DisplayName', 'R (dutch)');  % spiral Linear Model  
    hold on 
    R_s = Spir_res_dR(:, 3);
    plot(time_V, R_s, '--', 'DisplayName', 'R (Spiral)');  % spiral Linear Model 
end

hold on
plot(time_V, r, '-', 'DisplayName', 'r (Non-Linear)');                  % Non-Linear Model
title('r (1/sec^2)'); xlabel('t (sec)');
legend('show');
grid on
%%
figure(8)
if(dControl_latr(1) ~= 0)
    phi_ = (Lat_res_dA(:, 4) + phi0)*180/pi;
    plot(time_V, phi_, '--', 'DisplayName', '\phi (Full Linear lateral)');  % Full Lateral Linear Model  
    hold on
    phi_s = (Spir_res_dA(:, 4) + phi0)*180/pi;
    plot(time_V, phi_s, '--', 'DisplayName', '\phi (spiral)');  % spiral Linear Model  
elseif(dControl_latr(2) ~= 0)
      phi_ = (Lat_res_dR(:, 4) + phi0)*180/pi;
    plot(time_V, phi_, '--', 'DisplayName', '\phi (Full Linear lateral)');  % Full Linear Model  
    hold on
    phi_s = (Spir_res_dR(:, 4) + phi0)*180/pi;
    plot(time_V, phi_s, '--', 'DisplayName', '\phi (spiral)');  % Full Linear Model    
end

hold on
plot(time_V, phi_deg, '-', 'DisplayName', '\phi (Non-Linear)');                  % Non-Linear Model
title('\phi (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on
%%
figure(9)
if(dControl_latr(1) ~= 0)
    psi_ = (Lat_res_dA(:, 5) + psi0)*180/pi;
    plot(time_V, psi_, '--', 'DisplayName', '\psi (Full Linear lateral)');  % Full Linear Model  
 
elseif(dControl_latr(2) ~= 0)
      psi_ = (Lat_res_dR(:, 5) + psi0)*180/pi;
    plot(time_V, psi_, '--', 'DisplayName', '\psi (Full Linear lateral)');  % Full Linear Model  

end

hold on
plot(time_V, phi_deg, '-', 'DisplayName', '\psi (Non-Linear)');                  % Non-Linear Model
title('\psi (deg/sec)'); xlabel('t (sec)');
legend('show');
grid on

export_figure(max(double(get(groot,'Children')))+[-8:0], '', {'t1', 't2','t3','t4','t5','t6','t7','t8','t9'}, 600, {'emf', 'emf','emf','emf','emf','emf','emf','emf','emf'})
