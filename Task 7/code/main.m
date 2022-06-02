clc; clear; close all;

%% Inputs  
% Note: Run before Runing Simulink Model
% Initial AirPlane

plane = AirPlane("NT-33A_4.xlsx");
stability_matrix = plane.SM;

%% Initial Important Transfer Function
% Note: Run before Runing Simulink Model

servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1 , [1 0.1]);


%% Plotter Note: Run After running simulation to draw results
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
plot3(x,-y,-z);
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
plot(uvw.Time,-z)
title('z (ft)')
xlabel('time (sec)')
