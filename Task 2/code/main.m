clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia
[forces,Moments,Mass,I,timeSpan,dt,ICs] = Input();
g = 9.81;
Mg = Mass*g;
t_vec = timeSpan(1):dt:timeSpan(2);
n = length(t_vec);

%% Solving
Result = NaN(12,n);
Result(:,1) = ICs;
time = linspace(timeSpan(1), timeSpan(2), n);
for i =2:n
    Result(:, i) = RBDSolver(Result(:, i-1), dt);
end

% Rearranging Results
u_vec=Result(1,:);
v_vec=Result(2,:);
w_vec=Result(3,:);
p_vec=Result(4,:);
q_vec=Result(5,:);
r_vec=Result(6,:);
phi_vec=Result(7,:);
theta_vec=Result(8,:);
epsi_vec=Result(9,:);
xe_vec=Result(10,:);
ye_vec=Result(11,:);
ze_vec=Result(12,:);

%% Solving using ODE45

[t, states] = ode45(@DOF6,timeSpan,ICs);
t_ODE=t';
u_ODE=states(:,1)';
v_ODE=states(:,2)';
w_ODE=states(:,3)';
p_ODE=states(:,4)';
q_ODE=states(:,5)';
r_ODE=states(:,6)';
phi_ODE=states(:,7)';
theta_ODE=states(:,8)';
epsi_ODE=states(:,9)';
xe_ODE=states(:,10)';
ye_ODE=states(:,11)';
ze_ODE=states(:,12);

%% solving using simulink
%Run Smulink file
Sim_run=sim('simulink_test.slx');

%Extraxt Data from Simulink
%velocity body
Velocity_body=Sim_run.yout.getElement('velocity body');
%rotational speed body
Rotat_Velocity_body=Sim_run.yout.getElement('rotational speed body');
%rotational speed earth
Rotat_Velocity_earth=Sim_run.yout.getElement('rotational speed earth');
%positions earth
positions_earth=Sim_run.yout.getElement('positions earth');

t_sim=Velocity_body.Values.time;
u_sim=Velocity_body.Values.Data(:,1)';
v_sim=Velocity_body.Values.Data(:,2)';
w_sim=Velocity_body.Values.Data(:,3)';
p_sim=Rotat_Velocity_body.Values.Data(:,1)';
q_sim=Rotat_Velocity_body.Values.Data(:,2)';
r_sim=Rotat_Velocity_body.Values.Data(:,3)';
phi_sim=Rotat_Velocity_earth.Values.Data(:,1)';
theta_sim=Rotat_Velocity_earth.Values.Data(:,2)';
epsi_sim=Rotat_Velocity_earth.Values.Data(:,3)';
xe_sim=positions_earth.Values.Data(:,1)';
ye_sim=positions_earth.Values.Data(:,2)';
ze_sim=positions_earth.Values.Data(:,3)';

%% plots
%velocity body
figure
subplot(3, 1,1)
plot(t_vec,u_vec,'b',t_ODE,u_ODE,'g',t_sim,u_sim,'r')
legend({'$u(code)$','$u(ODE45)$','$u(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$u$','Interpreter','latex','FontSize',13)
title('u(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 2)
plot(t_vec,v_vec,'b',t_ODE,v_ODE,'g',t_sim,v_sim,'r')
legend({'$v(code)$','$v(ODE45)$','$v(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$v$','Interpreter','latex','FontSize',13)
title('v(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 3)
plot(t_vec,w_vec,'b',t_ODE,w_ODE,'g',t_sim,w_sim,'r')
legend({'$w(code)$','$w(ODE45)$','$w(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$w$','Interpreter','latex','FontSize',13)
title('w(from the code ,ODE45 and Simulink)')
grid on


%rotational speed body
figure
subplot(3, 1,1)
plot(t_vec,p_vec,'b',t_ODE,p_ODE,'g',t_sim,p_sim,'r')
legend({'$p(code)$','$p(ODE45)$','$p(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$p$','Interpreter','latex','FontSize',13)
title('p(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 2)
plot(t_vec,q_vec,'b',t_ODE,q_ODE,'g',t_sim,q_sim,'r')
legend({'$q(code)$','$q(ODE45)$','$q(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$q$','Interpreter','latex','FontSize',13)
title('q(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 3)
plot(t_vec,r_vec,'b',t_ODE,r_ODE,'g',t_sim,r_sim,'r')
legend({'$r(code)$','$r(ODE45)$','$r(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$r$','Interpreter','latex','FontSize',13)
title('r(from the code ,ODE45 and Simulink)')
grid on


%rotational speed earth
figure
subplot(3, 1,1)
plot(t_vec,phi_vec,'b',t_ODE,phi_ODE,'g',t_sim,phi_sim,'r')
legend({'$\phi(code)$','$\phi(ODE45)$','$\phi(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$\phi$','Interpreter','latex','FontSize',13)
title('\phi(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 2)
plot(t_vec,theta_vec,'b',t_ODE,theta_ODE,'g',t_sim,theta_sim,'r')
legend({'$\theta(code)$','$\theta(ODE45)$','$\theta(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$\theta$','Interpreter','latex','FontSize',13)
title('\theta(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 3)
plot(t_vec,epsi_vec,'b',t_ODE,epsi_ODE,'g',t_sim,epsi_sim,'r')
legend({'$\psi(code)$','$\psi(ODE45)$','$\psi(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$\psi$','Interpreter','latex','FontSize',13)
title('\psi(from the code ,ODE45 and Simulink)')
grid on


%positions earth
figure
subplot(3, 1,1)
plot(t_vec,xe_vec,'b',t_ODE,xe_ODE,'g',t_sim,xe_sim,'r')
legend({'$xe(code)$','$xe(ODE45)$','$xe(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$xe$','Interpreter','latex','FontSize',13)
title('xe(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 2)
plot(t_vec,ye_vec,'b',t_ODE,ye_ODE,'g',t_sim,ye_sim,'r')
legend({'$ye(code)$','$ye(ODE45)$','$ye(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$ye$','Interpreter','latex','FontSize',13)
title('ye(from the code ,ODE45 and Simulink)')
grid on
subplot(3,1, 3)
plot(t_vec,ze_vec,'b',t_ODE,ze_ODE,'g',t_sim,ze_sim,'r')
legend({'$ze(code)$','$ze(ODE45)$','$ze(sim)$'},'Location','southeast','FontSize',8,...
    'Interpreter','latex')
xlabel('$t$','Interpreter','latex','FontSize',13)
ylabel('$ze$','Interpreter','latex','FontSize',13)
title('ze(from the code ,ODE45 and Simulink)')
grid on
