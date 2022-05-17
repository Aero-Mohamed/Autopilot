clc; clear; close all;

%% Inputs 
% Initial AirPlane
plane = AirPlane("excelsheet_data_modified.xlsx");
stability_matrix = plane.SM;

%% Plotter
u = uvw.Data(:,1);
v = uvw.Data(:,2);
w = uvw.Data(:,3);
p = pqr.Data(:,1);

q = pqr.Data(:,2);
r = pqr.Data(:,3);
phi = phi_theta_psi.Data(:,1);
theta = phi_theta_psi.Data(:,2);
psi = phi_theta_psi.Data(:,3);
x = xyz.Data(:, 1);
y = xyz.Data(:, 2);
z = xyz.Data(:, 3);

beta_deg= beta.Data(:,1) * 180/pi;
alpha_deg = atan(w./u)*180/pi;
p_deg = p*180/pi;
q_deg = q*180/pi;
r_deg = r*180/pi;
phi_deg = phi*180/pi;
theta_deg = theta*180/pi;
psi_deg = psi*180/pi;

figure
plot3(x,y,z);
title('Trajectory')
figure
subplot(4,3,1)
plot(uvw.Time,u)
title('u (ft/sec)')
xlabel('time (sec)')
subplot(4,3,2)
plot(uvw.Time,beta_deg)
title('\beta (deg)')
xlabel('time (sec)')
subplot(4,3,3)
plot(uvw.Time,alpha_deg)
title('\alpha (deg)')
xlabel('time (sec)')
subplot(4,3,4)
plot(uvw.Time,p_deg)
title('p (deg/sec)')
xlabel('time (sec)')
subplot(4,3,5)
plot(uvw.Time,q_deg)
title('q (deg/sec)')
xlabel('time (sec)')
subplot(4,3,6)
plot(uvw.Time,r_deg)
title('r (deg/sec)')
xlabel('time (sec)')
subplot(4,3,7)
plot(uvw.Time,phi_deg)
title('\phi (deg)')
xlabel('time (sec)')
subplot(4,3,8)
plot(uvw.Time,theta_deg)
title('\theta (deg)')
xlabel('time (sec)')
subplot(4,3,9)
plot(uvw.Time,psi_deg)
title('\psi (deg)')
xlabel('time (sec)')
subplot(4,3,10)
plot(uvw.Time,x)
title('x (ft)')
xlabel('time (sec)')
subplot(4,3,11)
plot(uvw.Time,y)
title('y (ft)')
xlabel('time (sec)')
subplot(4,3,12)
plot(uvw.Time,z)
title('z (ft)')
xlabel('time (sec)')


%% Initial Important Variables
steps = (plane.timeSpan(2) - plane.timeSpan(1))/plane.dt;
Result = NaN(12, steps);
Result(:,1) = plane.ICs;
time_V = linspace(0, plane.timeSpan(2), steps+1);

%% Initial Important Transfer Function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);

%%
%%%%%%%%%%%%%%%%%%% Lateral Full Linear Model %%%%%%%%%%%%%%%%%%%
[A_lat, B_lat, C_lat, D_lat] = plane.lateralFullLinearModel();
LatSS = ss(A_lat, B_lat, C_lat, D_lat);
LatTF = tf(LatSS);

%% Design a “Yaw damper” for the Dutch roll mode
R_DR_L = LatTF(3, 2);
OL_r_rcom=servo*R_DR_L;

yawDamperControldesignValues = matfile("DesignValues/yawDamperControlDesignValues-Design2.mat");

yaw_C_tf = yawDamperControldesignValues.C;
CL_r_rcom_tf = tf(yawDamperControldesignValues.IOTransfer_r2y);
yaw_C_action_tf = tf(yawDamperControldesignValues.IOTransfer_r2u);


figure;
impulse(yaw_C_action_tf);
title('r/r_{com} Control Action');
figure;
impulse(CL_r_rcom_tf);

%% Design a "Roll Controller"

LatSSYawDamped = feedback(servo * LatSS, yaw_C_tf, 2, 3, 1);
LatTFYawDamped = tf(LatSSYawDamped);
checking_r_rcom_tf = LatTFYawDamped(3, 2);
hold on
impulse(checking_r_rcom_tf, 'r--');
title('r/r_{com} - With Controller Vs. New Lat SS');
hold off

phi_da = minreal(servo * LatTFYawDamped(4, 1));

rollControldesignValues = matfile("DesignValues/rollControllerValues.mat");

roll_C_tf = rollControldesignValues.C;
CL_roll_tf = tf(rollControldesignValues.IOTransfer_r2y);
roll_C_action_tf = tf(rollControldesignValues.IOTransfer_r2u);



%% 
%%%%%%%%%%%%%%%%%%% Longitudenal Full Linear Model %%%%%%%%%%%%%%%%%%%
% Two Inputs - Four Output Each
[A_long, B_long, C_long, D_long] = plane.fullLinearModel();
LongSS = ss(A_long, B_long, C_long, D_long);
LongTF = tf(LongSS);

%% pitch control theta/theta_com
theta_dE = LongTF(4,1);
OL_theta_thetacom = -servo * theta_dE;
pitchControldesignValues = matfile("DesignValues/pitchControldesignValues.mat");

pitch_PD_tf = pitchControldesignValues.C2;
pitch_PI_tf = pitchControldesignValues.C1;
CL_theta_thetacom_tf = tf(pitchControldesignValues.IOTransfer_r2y);
pitch_C_action_tf = tf(pitchControldesignValues.IOTransfer_r2u);

% figure;
% step(CL_theta_thetacom_tf)
% figure;
% step(pitch_C_action_tf)

%% Velocity Controller u/u_com
u_dTh = LongTF(1, 2);
OL_u_ucom = u_dTh * servo * engine_timelag;
velocityControldesignValues = matfile("DesignValues/velocityControldesignValues.mat");

velocity_C2_tf = velocityControldesignValues.C2;
velocity_C1_tf = velocityControldesignValues.C1;
CL_u_ucom_tf = tf(velocityControldesignValues.IOTransfer_r2y);
velocity_C_action_tf = tf(velocityControldesignValues.IOTransfer_r2u);

% figure;
% step(CL_u_ucom_tf)
% figure;
% step(velocity_C_action_tf)

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

% figure;
% step(CL_h_thetacom_tf)
% figure;
% step(altitude_C_action_tf)

