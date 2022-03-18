% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
clc
clear All
close all
%  Excel Sheets Data
% global aircraft_derivatives_dimensions 
filename_density_L = 'NT-33A_4.xlsx'; %%put here the location of your excel sheet

aircraft_data=xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61

%%in the next step we will read from the vector(aircraft_data) but take care of the order the values in excel sheet is arranged
% initial conditions
s0 = aircraft_data(4:15);

% control actions values
% da = aircraft_data(57);
% dr = aircraft_data(58);
% de = aircraft_data(59);
% dth = aircraft_data(60);
dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];

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

U0=s0(1); W0=s0(3); TH0=s0(8);
% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);

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

%% PHUGOID MODE (LONG PERIOD MODE)
A_phug=[XU -g*cos(TH0)
    ZU/(U0+ZQ) g*sin(TH0)];
B_phug=[XDE XDTH
    ZDE/(ZQ+U0) ZDTH/(ZQ+U0)];
C_phug=eye(2);D_phug=zeros(2,2);

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
C_short=eye(2);D_short=zeros(2,2);

SHORT_SS=ss(A_short,B_short,C_short,D_short);
SHORT_TF=tf(SHORT_SS);

W_DE_PH=SHORT_TF(1,1);
W_DTH_PH=SHORT_TF(1,2);
Q_DE_PH=SHORT_TF(2,1);
Q_DTH_PH=SHORT_TF(2,2);

%% root locus and bode plots 

%% full linear modle

%u/de
figure()
rlocus(LONGT_TF(1,1)) 
title('LINEAR FULL MODLE root locus (u/\delta_{e}) ')

figure ()
bode(LONGT_TF(1,1))
title('LINEAR FULL MODLE bode plot (u/\delta_{e}) ')
grid on 

%u/d_th
figure()
rlocus(LONGT_TF(1,2))
title('LINEAR FULL MODLE root locus (u/\delta_{e}) ')

figure ()
bode(LONGT_TF(1,2))
title('LINEAR FULL MODLE bode plot (u/\delta_{e}) ')
grid on 

%w/de
figure()
rlocus(LONGT_TF(2,1)) 
title('LINEAR FULL MODLE root locus (w/\delta_{e}) ')

figure ()
bode(LONGT_TF(2,1))
title('LINEAR FULL MODLE bode plot (w/\delta_{e}) ')
grid on 

%w/d_th
figure()
rlocus(LONGT_TF(2,2))
title('LINEAR FULL MODLE root locus (w/\delta_{e}) ')

figure ()
bode(LONGT_TF(2,2))
title('LINEAR FULL MODLE bode plot (w/\delta_{e}) ')
grid on 

%q/de
figure()
rlocus(LONGT_TF(3,1)) 
title('LINEAR FULL MODLE root locus (q/\delta_{e}) ')

figure ()
bode(LONGT_TF(3,1))
title('LINEAR FULL MODLE bode plot (q/\delta_{e}) ')
grid on 

%q/d_th
figure()
rlocus(LONGT_TF(3,2))
title('LINEAR FULL MODLE root locus (q/\delta_{e}) ')

figure ()
bode(LONGT_TF(3,2))
title('LINEAR FULL MODLE bode plot (q/\delta_{e}) ')
grid on 

%theta/de
figure()
rlocus(LONGT_TF(4,1)) 
title('LINEAR FULL MODLE root locus (\theta/\delta_{e}) ')

figure ()
bode(LONGT_TF(4,1))
title('LINEAR FULL MODLE bode plot (\theta/\delta_{e}) ')
grid on 

%theta/d_th
figure()
rlocus(LONGT_TF(4,2))
title('LINEAR FULL MODLE root locus (\theta/\delta_{e}) ')

figure ()
bode(LONGT_TF(4,2))
title('LINEAR FULL MODLE bode plot (\theta/\delta_{e}) ')
grid on 

%% long period mode (approximate)

%u/de
figure()
rlocus(PHUG_TF(1,1)) 
title('LONG PERIOD MODE root locus (u/\delta_{e}) ')

figure ()
bode(PHUG_TF(1,1))
title('LONG PERIOD MODE bode plot (u/\delta_{e}) ')
grid on 

%u/d_th
figure()
rlocus(PHUG_TF(1,2))
title('LONG PERIOD MODE root locus (u/\delta_{e}) ')

figure ()
bode(PHUG_TF(1,2))
title('LONG PERIOD MODE bode plot (u/\delta_{e}) ')
grid on 

%theta/de
figure()
rlocus(PHUG_TF(2,1)) 
title('LONG PERIOD MODE root locus (\theta/\delta_{e}) ')

figure ()
bode(PHUG_TF(2,1))
title('LONG PERIOD MODE bode plot (\theta/\delta_{e}) ')
grid on 

%theta/d_th
figure()
rlocus(PHUG_TF(2,2))
title('LONG PERIOD MODE root locus (\theta/\delta_{e}) ')

figure ()
bode(PHUG_TF(2,2))
title('LONG PERIOD MODE bode plot (\theta/\delta_{e}) ')
grid on 

%% short period mode (approximate)

%w/de
figure()
rlocus(SHORT_TF(1,1)) 
title('LONG PERIOD MODE root locus (w/\delta_{e}) ')

figure ()
bode(SHORT_TF(1,1))
title('LONG PERIOD MODE bode plot (w/\delta_{e}) ')
grid on 

%w/d_th
figure()
rlocus(SHORT_TF(1,2))
title('LONG PERIOD MODE root locus (w/\delta_{e}) ')

figure ()
bode(SHORT_TF(1,2))
title('LONG PERIOD MODE bode plot (w/\delta_{e}) ')
grid on 

%q/de
figure()
rlocus(SHORT_TF(2,1)) 
title('LONG PERIOD MODE root locus (q/\delta_{e}) ')

figure ()
bode(SHORT_TF(2,1))
title('LONG PERIOD MODE bode plot (q/\delta_{e}) ')
grid on 

%q/d_th
figure()
rlocus(SHORT_TF(2,2))
title('LONG PERIOD MODE root locus (q/\delta_{e}) ')

figure ()
bode(SHORT_TF(2,2))
title('LONG PERIOD MODE bode plot (q/\delta_{e}) ')
grid on 
