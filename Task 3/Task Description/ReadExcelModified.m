% the aerodynamic forces and moments can be expressed as a function of all the motion variables [Nelson] page 63
% clc
% clear all
% close all
%  Excel Sheets Data
% global aircraft_derivatives_dimensions 
filename_density_L = 'excelsheet_data_modified'; %%put here the location of your excel sheet

aircraft_data=xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61

%%in the next step we will read from the vector(aircraft_data) but take care of the order the values in excel sheet is arranged

% Time vector parameters
dt = aircraft_data(1);    tfinal = aircraft_data(2); lengths=tfinal/dt+1;
time_V = (0:dt:tfinal)';

% initial conditions
s0 = aircraft_data(4:15);
sdot0 = zeros(12,1);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);    % Vto

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
SD_Long_final = SD_Long;


% stability derivatives Lateral motion
SD_Lat_dash = aircraft_data(37:50);
run('LateralFunc');
SD_Lat_final = [ SD_Lat(1) ; SD_Lat(3) ; SD_Lat(4) ; SD_Lat(5) ; SD_Lat(6) ;...
                 SD_Lat(7) ; SD_Lat(8) ; SD_Lat(9:10) ; SD_Lat(11) ; SD_Lat(12) ;...
                  SD_Lat(13) ; SD_Lat(14) ];
              
% initial gravity force
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];

