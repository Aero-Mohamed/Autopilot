% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
clc
clear All
close all


% Servo Transfer Function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);

%  Excel Sheets Data
% global aircraft_derivatives_dimensions 
filename_density_L = 'NT-33A_4.xlsx'; %%put here the location of your excel sheet

aircraft_data=xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61

%%in the next step we will read from the vector(aircraft_data) but take care of the order the values in excel sheet is arranged
% initial conditions
s0 = aircraft_data(4:15);
sdot0 = zeros(12,1);
IC=s0;
% control actions values
dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];
da = dc(1);
dr = dc(2);
de = dc(3);
dth = dc(4);

% gravity, mass % inertia
m = aircraft_data(51);
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);    Ixy=0;  Iyz=0;
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy , -Iyz ;...
    -Ixz , -Iyz , Izz];
invI=inv(I);

% stability derivatives Longitudinal motion
SD_Long = aircraft_data(21:36);
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

U0=s0(1);V0=s0(2); W0=s0(3); TH0=s0(8); z0=s0(12);
Vt0 = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);    % Vto

% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);

Yp=0;
Yr=0;
YDa_star=SD_Lat_dash(9); 
YDr_star=SD_Lat_dash(10);
Yb=SD_Lat_dash(2); 
YDa=SD_Lat_dash(9)*Vt0; 
YDr=SD_Lat_dash(10)*Vt0;

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

% initial gravity force
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];


% lat_undashed

SD_Lat(1:2) = SD_Lat_dash(1:2); % Yv , YB 
G = 1 / (1 - Ixz^2/Ixx/Izz); 
a = [G, G*Ixz/Ixx;...
     G*Ixz/Izz, G];
 
SD_Lat(3:4) = a \ SD_Lat_dash(3:4); % LB , NB 
SD_Lat(3:4) = SD_Lat(3:4) / Vt0 ;   % Lv , Nv 
SD_Lat(5:6) = a \ SD_Lat_dash(5:6); % Lp , Np 
SD_Lat(7:8) = a \ SD_Lat_dash(7:8); % Lr , Nr 
SD_Lat(9:10) = Vt0*SD_Lat_dash(9:10); % Yda , Ydr 
SD_Lat(11:12) = a \ SD_Lat_dash(11:12); % Lda , Nda 
SD_Lat(13:14) = a \ SD_Lat_dash(13:14); % Ldr , Ndr 


SD_Lat = SD_Lat';  % Yv , YB , Lv , Nv , Lp , Np , Lr , Nr , Yda , Ydr , Lda , Nda , Ldr , Ndr 

U_Yv=SD_Lat(1);
U_YB=SD_Lat(2);
U_Lv=SD_Lat(3);
U_Nv=SD_Lat(4);
U_Lp=SD_Lat(5);
U_Np=SD_Lat(6);
U_Lr=SD_Lat(7);
U_Nr=SD_Lat(8);
U_Yda=SD_Lat(9);
U_Ydr=SD_Lat(10);
U_Lda=SD_Lat(11);
U_Nda=SD_Lat(12);
U_Ldr=SD_Lat(13);
U_Ndr=SD_Lat(14);


%stability matrix
%u v w...p q r...w_dot...da dr de dth 

stability_matrix=[XU 0 XW 0 0 0 0 0 0 XDE XDTH;...
    0 U_Yv 0 Yp 0 Yr 0 U_Yda U_Ydr 0 0;...        
    ZU 0 ZW 0 ZQ 0 ZWD 0 0 ZDE ZDTH;...
    0 U_Lv 0 U_Lp 0 U_Lr 0 U_Lda U_Ldr 0 0;...
    MU 0 MW 0 MQ 0 MWD 0 0 MDE MDTH;...
    0 U_Nv 0 U_Np 0 U_Nr 0 U_Nda U_Ndr 0 0];


%% TF

% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
clc
clear All
close all


% Servo Transfer Function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);

%  Excel Sheets Data
% global aircraft_derivatives_dimensions 
filename_density_L = 'NT-33A_4.xlsx'; %%put here the location of your excel sheet

aircraft_data=xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61

%%in the next step we will read from the vector(aircraft_data) but take care of the order the values in excel sheet is arranged
% initial conditions
s0 = aircraft_data(4:15);
IC=s0;
% control actions values
dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];
da = dc(1);
dr = dc(2);
de = dc(3);
dth = dc(4);

% gravity, mass % inertia
m = aircraft_data(51);
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);    Ixy=0;  Iyz=0;
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy , -Iyz ;...
    -Ixz , -Iyz , Izz];
invI=inv(I);

% stability derivatives Longitudinal motion
SD_Long = aircraft_data(21:36);
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

U0=s0(1); W0=s0(3); TH0=s0(8); z0=s0(12);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);    % Vto

% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);

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

% initial gravity force
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];

% liniearized set of Longitudinal equs
A_longt=[XU XW -W0 -g*cos(TH0)
    ZU/(1-ZWD) ZW/(1-ZWD) (ZQ+U0)/(1-ZWD) -g*sin(TH0)/(1-ZWD)
    MU+MWD*ZU/(1-ZWD) MW+MWD*ZW/(1-ZWD) MQ+MWD*(ZQ+U0)/(1-ZWD) -MWD*g*sin(TH0)/(1-ZWD)
    0 0 1 0];
B_longt=[XDE XDTH
    ZDE/(1-ZWD) ZDTH/(1-ZWD)
    MDE+MWD*ZDE/(1-ZWD) MDTH+MWD*ZDTH/(1-ZWD)
    0 0];
C_longt=eye(4); D_longt=zeros(4,2);

LONGT_SS=ss(A_longt,B_longt,C_longt,D_longt);
LONGT_TF=tf(LONGT_SS);

U_DE_F=LONGT_TF(1,1);
U_DTH_F=LONGT_TF(1,2);
W_DE_F=LONGT_TF(2,1);
W_DTH_F=LONGT_TF(2,2);
Q_DE_F=LONGT_TF(3,1);
Q_DTH_F=LONGT_TF(3,2);
THETA_DE_F=LONGT_TF(4,1);
THETA_DTH_F=LONGT_TF(4,2);

%%%%%%%
Wdot_DE_F=W_DE_F*differentiator;
%az
%%%%%%%
% PHUGOID MODE (LONG PERIOD MODE)
A_phug=[XU -g*cos(TH0)
    -ZU/(U0+ZQ) g*sin(TH0)];
B_phug=[XDE XDTH
    -ZDE/(ZQ+U0) -ZDTH/(ZQ+U0)];
C_phug=eye(2);
D_phug=zeros(2,2);

PHUG_SS=ss(A_phug,B_phug,C_phug,D_phug);
PHUG_TF=tf(PHUG_SS);

U_DE_PH=PHUG_TF(1,1);
U_DTH_PH=PHUG_TF(1,2);
THETA_DE_PH=PHUG_TF(2,1);
THETA_DTH_PH=PHUG_TF(2,2);

% SHORT PERIOD MODE
A_short=[ZW/(1-ZWD) (ZQ+U0)/(1-ZWD)
    (MW+ZW*MWD/(1-ZWD)) (MQ+MWD*(ZQ+U0)/(1-ZWD))];
B_short=[ZDE/(1-ZWD) ZDTH
    MDE+MWD*ZDE/(1-ZWD) MDTH+MWD*ZDTH/(1-ZWD)];
C_short=eye(2);
D_short=zeros(2,2);

SHORT_SS=ss(A_short,B_short,C_short,D_short);
SHORT_TF=tf(SHORT_SS);

W_DE_SH=SHORT_TF(1,1);
W_DTH_SH=SHORT_TF(1,2);
Q_DE_SH=SHORT_TF(2,1);
Q_DTH_SH=SHORT_TF(2,2);

% Liniearized set of Lateral Equation
A_Lat=[Yb/Vto (Yp+W0)/Vto (Yr-U0)/Vto g*cos(TH0)/Vto 0;...
       Lbd Lpd Lrd 0 0;...
       Nbd Npd Nrd 0 0;...
       0 1 tan(TH0) 0 0;...
       0 0 1/cos(TH0) 0 0];
B_Lat=[YDa_star YDr_star;...
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

% 3DOF Spiral Mode Approximation
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


% 2DOF Dutch Mode Approximation
A_Dutch = [Yb/Vto (Yr-U0)/Vto-tan(TH0)*(Yp+W0)/Vto;Nbd Nrd-tan(TH0)*Npd];
B_Dutch = [YDa_star YDr_star;NDad NDrd];
C_Dutch = eye(2); D_Dutch = zeros(2,2);

Dutch_SS = ss(A_Dutch, B_Dutch, C_Dutch, D_Dutch);
Dutch_TF = tf(Dutch_SS);

R_DA_D = Dutch_TF(1, 1);
R_DR_D = Dutch_TF(1, 2);

B_DA_D = Dutch_TF(2, 1);
B_DR_D = Dutch_TF(2, 2);


% 1DOF Roll Approximation 
A_Roll = Lpd;
B_Roll = LDad;
C_Roll = eye(1); D_Roll = zeros(1, 1);

Roll_SS = ss(A_Roll, B_Roll, C_Roll, D_Roll);
Roll_TF = tf(Roll_SS);

P_DA_R = Roll_TF(1 ,1);

%% simulation results and plots

t_s = sim_results.Time;
x_s = sim_results.Data(:,1);
y_s = - sim_results.Data(:,2);
z_s = - sim_results.Data(:,3);

phi_s = sim_results.Data(:,4);
theta_s = sim_results.Data(:,5);
psi_s = sim_results.Data(:,6);

u_s = sim_results.Data(:,7);
v_s = sim_results.Data(:,8);
w_s = sim_results.Data(:,9);

p_s = sim_results.Data(:,10);
q_s = sim_results.Data(:,11);
r_s = sim_results.Data(:,12);

ud_s = sim_results.Data(:,13);
vd_s = sim_results.Data(:,14);
wd_s = sim_results.Data(:,15);

beta_deg_s=asin(v_s/Vt0)*180/pi;
alpha_deg_s=atan(w_s./u_s)*180/pi;
p_deg_s=p_s*180/pi;
q_deg_s=q_s*180/pi;
r_deg_s=r_s*180/pi;
phi_deg_s=phi_s*180/pi;
theta_deg_s=theta_s*180/pi;
psi_deg_s=psi_s*180/pi;

% plots

figure
plot3(x_s,y_s,z_s);
title('Trajectory')
grid on

figure
plot(x_s,y_s);
title('Trajectory (x,y)')
grid on

figure
plot(y_s,z_s);
title('Trajectory (y,z)')
grid on

figure
plot(x_s,z_s);
title('Trajectory (x,z)')
grid on

figure
subplot(4,3,1)
plot(t_s,u_s)
title('u (ft/sec)')
xlabel('time (sec)')
subplot(4,3,2)
plot(t_s,beta_deg_s)
title('\beta (deg)')
xlabel('time (sec)')
subplot(4,3,3)
plot(t_s,alpha_deg_s)
title('\alpha (deg)')
xlabel('time (sec)')
subplot(4,3,4)
plot(t_s,p_deg_s)
title('p (deg/sec)')
xlabel('time (sec)')
subplot(4,3,5)
plot(t_s,q_deg_s)
title('q (deg/sec)')
xlabel('time (sec)')
subplot(4,3,6)
plot(t_s,r_deg_s)
title('r (deg/sec)')
xlabel('time (sec)')
subplot(4,3,7)
plot(t_s,phi_deg_s)
title('\phi (deg)')
xlabel('time (sec)')
subplot(4,3,8)
plot(t_s,theta_deg_s)
title('\theta (deg)')
xlabel('time (sec)')
subplot(4,3,9)
plot(t_s,psi_deg_s)
title('\psi (deg)')
xlabel('time (sec)')
subplot(4,3,10)
plot(t_s,x_s)
title('x (ft)')
xlabel('time (sec)')
subplot(4,3,11)
plot(t_s,y_s)
title('y (ft)')
xlabel('time (sec)')
subplot(4,3,12)
plot(t_s,z_s)
title('z (ft)')
xlabel('time (sec)')
