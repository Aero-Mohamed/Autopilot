clc; clear; close all;

%% Inputs 
% Forces, Moments and Inertia

[Mass, g, I, timeSpan, dt, ICs, ICs_dot0, Vt0, ...
    dc, SD_Long, SD_Lat, initialGravity] = Input("excelsheet_data_modified.xlsx");

%% Solving
steps = (timeSpan(2) - timeSpan(1))/dt;
Result = NaN(12, steps);
Result(:,1) = ICs;
time = linspace(0, 15, steps);
for i =2:steps
    Result(:, i) = RBDSolver(Result(:, i-1), dt);
end


