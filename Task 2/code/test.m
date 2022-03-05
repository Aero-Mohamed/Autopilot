clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia
Forces = [10 5 9]';         % Vector
Moments = [10 20 5]';       % Vector
Mass = 15;
I = [1 -2 -1;
    -2 5 -3;
    -1 -3 0.1];
% Integration time span & Step
timeSpan = [0 15];
dt = 0.001;
% Initial Conditions
ICs = [10; 2; 0; 2*pi/180; pi/180; 0; 20*pi/180; 15*pi/180; 30*pi/180; 2; 4; 7];

%% Solving
steps = (timeSpan(2) - timeSpan(1))/dt;
Result = NaN(12, steps);
Result(:,1) = ICs;
time = linspace(0, 15, steps);
for i =2:steps
    Result(:, i) = RBDSolver(Result(:, i-1), dt);
end

%% Solving using ODE45
[t, states] = ode45(@DOF6,timeSpan,ICs);
