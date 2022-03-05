function [Forces,Moments,Mass,I,timeSpan,dt,ICs] = Input()
    % Inputs 
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
    % [u; v; w; p; q; r; phi; theta; epsi; xe0; ye0; ze0]
    ICs = [10; 2; 0; 2*pi/180; pi/180; 0; 20*pi/180; 15*pi/180; 30*pi/180; 2; 4; 7];

end

