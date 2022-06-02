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

%% liniearized set of Longitudinal equs
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
%% PHUGOID MODE (LONG PERIOD MODE)
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

%% SHORT PERIOD MODE
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


%% Liniearized set of Lateral Equation
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

R_DA_D = Dutch_TF(1, 1);
R_DR_D = Dutch_TF(1, 2);

B_DA_D = Dutch_TF(2, 1);
B_DR_D = Dutch_TF(2, 2);


%% 1DOF Roll Approximation 
A_Roll = Lpd;
B_Roll = LDad;
C_Roll = eye(1); D_Roll = zeros(1, 1);

Roll_SS = ss(A_Roll, B_Roll, C_Roll, D_Roll);
Roll_TF = tf(Roll_SS);

P_DA_R = Roll_TF(1 ,1);
