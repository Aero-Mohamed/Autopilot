clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia
plane = AirPlane("NT-33A_4.xlsx");

steps = (plane.timeSpan(2) - plane.timeSpan(1))/plane.dt;
Result = NaN(12, steps);
Result(:,1) = plane.ICs;
time_V = linspace(0, plane.timeSpan(2), steps+1);

%% Lateral Full Linear Model 
[A_lat, B_lat, C_lat, D_lat] = plane.lateralFullLinearModel();
LatSS = ss(A_lat, B_lat, C_lat, D_lat);
LatTF = tf(LatSS);

%% Longitudenal Full Linear Model

% Two Inputs - Four Output Each
[A_long, B_long, C_long, D_long] = plane.fullLinearModel();
LongSS = ss(A_long, B_long, C_long, D_long);
LongTF = tf(LongSS);

%% Servo Transfer Function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);

%% pitch control theta/theta_com
theta_dE = LongTF(4,1);
OL_theta_thetacom = -servo * theta_dE;
pitchControldesignValues = matfile("DesignValues/pitchControldesignValues.mat");

pitch_PD_tf = pitchControldesignValues.C2;
pitch_PI_tf = pitchControldesignValues.C1;
CL_theta_thetacom_tf = tf(pitchControldesignValues.IOTransfer_r2y);
pitch_C_action_tf = tf(pitchControldesignValues.IOTransfer_r2u);

figure;
step(CL_theta_thetacom_tf)
figure;
step(pitch_C_action_tf)

%% Velocity Controller u/u_com
u_dTh = LongTF(1, 2);
OL_u_ucom = u_dTh * servo * engine_timelag;
velocityControldesignValues = matfile("DesignValues/velocityControldesignValues.mat");

velocity_C2_tf = velocityControldesignValues.C2;
velocity_C1_tf = velocityControldesignValues.C1;
CL_u_ucom_tf = tf(velocityControldesignValues.IOTransfer_r2y);
velocity_C_action_tf = tf(velocityControldesignValues.IOTransfer_r2u);

figure;
step(CL_u_ucom_tf)
figure;
step(velocity_C_action_tf)

%% Altitude Controller h/thetacom
w_de = LongTF(2,1);
theta_de = LongTF(4, 1);
w_theta = minreal(w_de/theta_de);
h_theta = -1 * integrator * (w_theta - plane.u0);
OL_h_thetacom = minreal(CL_theta_thetacom_tf * h_theta);

altitudeControldesignValues = matfile("DesignValues/altitudeControldesignValues.mat");

altitude_C2_tf = altitudeControldesignValues.C2;
altitude_C1_tf = altitudeControldesignValues.C1;
CL_h_thetacom_tf = tf(altitudeControldesignValues.IOTransfer_r2y);
altitude_C_action_tf = tf(altitudeControldesignValues.IOTransfer_r2u);

figure;
step(CL_h_thetacom_tf)
figure;
step(altitude_C_action_tf)

