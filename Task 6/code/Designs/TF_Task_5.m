% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
clean_check=input('2 to clear and close 1 to close only 0 to continue and run for first time >>>\n');
if clean_check==2
    clc
    clearvars
    close all
elseif clean_check==1
    close all
end

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


rlocus(Q_DE_F)
grid on

p=pole(Q_DE_F);
[short_poles_abs, i_short]=max(abs(p));
[long_poles_abs, i_long]=min(abs(p));
zeta_short=abs(real(p(i_short)))/short_poles_abs
zeta_long=abs(real(p(i_long)))/long_poles_abs

%% Servo Transfer Function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);
%% pitch control theta/theta_com
% Open Loop TFs
OL_theta_thetacom = -servo * THETA_DE_F;

%% siso tool for pitch 
siso_tool=input('1 to open siso tool for (pitch) or 0 to continue without open it >>>\n');
if siso_tool==1
   sisotool('pitchDesignerSession.mat')
end

%% after exporting from siso tool (pitch)
x=input('1 to continuo or 0 to stop (pitch) >>>\n');
if x==0
    
else
    PD_tf_pch=C2_pch;
    PI_tf_pch=C1_pch;
    CL_theta_thetacom_tf=tf(IOTransfer_r2y_pch);
    Con_action_tf_pch=tf(IOTransfer_r2u_pch);
    f1=figure;
    step(CL_theta_thetacom_tf)
    f2=figure;
    step(Con_action_tf_pch)
end

%% velocity control
Ol_u_ucom=U_DTH_F*engine_timelag*servo;

%% siso tool for velocity
siso_tool=input('1 to open siso tool for (velocity) or 0 to continue without open it >>>\n');
if siso_tool==1
   sisotool('velocityDesignerSession.mat')
end

%% after exporting from siso tool (velocity)
x=input('1 to continuo or 0 to stop (velocity) >>>\n');
if x==0
    
else
    PD_tf_vel=C2_vel;
    PI_tf_vel=C1_vel;
    CL_u_ucom_tf=tf(IOTransfer_r2y_vel);
    Con_action_tf_vel=tf(IOTransfer_r2u_vel);
    f1=figure;
    step(CL_u_ucom_tf)
    f2=figure;
    step(Con_action_tf_vel)
end

%% altitude Control
h_theta=-integrator*(minreal(W_DE_F/THETA_DE_F)-U0);

%% siso tool for altitude
siso_tool=input('1 to open siso tool for (altitude) or 0 to continue without open it >>>\n');
if siso_tool==1
   sisotool('altitudeDesignerSession.mat')
end

%% after exporting from siso tool (altitude)
x=input('1 to continuo or 0 to stop (altitude) >>>\n');
if x==0
    
else
    PD_tf_alt=C2_alt;
    PI_tf_alt=C1_alt;
    CL_h_hcom_tf=tf(IOTransfer_r2y_alt);
    Con_action_tf_alt=tf(IOTransfer_r2u_alt);
    f1=figure;
    step(CL_h_hcom_tf)
    f2=figure;
    step(Con_action_tf_alt)
end
