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

%% D. Test the “Altitude Hold” controller and compare the response with the same test on the State space model
D_linear = load('./Results/linear_simulation_1000ft_altitude_hold.mat');
D_non_linear = load('./Results/nonlinear_simulation_1000ft_altitude_hold.mat');

figure
plot(D_linear.delta_theta.Time, D_linear.delta_theta.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.delta_theta.Time, D_non_linear.delta_theta.Data, '-r', 'LineWidth', 2);
xlim([0 40]);
legend('Linear Model', 'Non Linear Model');
title('{\delta}{\theta}');

figure
plot(D_linear.delta_u.Time, D_linear.delta_u.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.delta_u.Time, D_non_linear.delta_u.Data, '-r', 'LineWidth', 2);
legend('Linear Model', 'Non Linear Model');
title('{\delta}{u}');

figure
plot(D_linear.gamma.Time, D_linear.gamma.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.gamma.Time, D_non_linear.gamma.Data, '-r', 'LineWidth', 2);
xlim([0 40]);
legend('Linear Model', 'Non Linear Model');
title('{\delta}{\gamma}');

figure
plot(D_linear.altitude.Time, D_linear.altitude.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.altitude.Time, D_non_linear.altitude.Data, '-r', 'LineWidth', 2);
xlim([0 40]);
legend('Linear Model', 'Non Linear Model');
title('{altitude} {ft}');

figure
plot(D_linear.delta_E.Time, D_linear.delta_E.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.delta_E.Time, D_non_linear.delta_E.Data, '-r', 'LineWidth', 2);
xlim([0 40]);
legend('Linear Model', 'Non Linear Model');
title('{\delta}{Elevetor}');

figure
plot(D_linear.delta_TH.Time, D_linear.delta_TH.Data, '--b', 'LineWidth', 2);
hold on
plot(D_non_linear.delta_TH.Time, D_non_linear.delta_TH.Data, '-r', 'LineWidth', 2);
xlim([0 40]);
legend('Linear Model', 'Non Linear Model');
title('{\delta}{Thrust}');


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
plot(uvw.Time,z)
title('z (ft)')
xlabel('time (sec)')
