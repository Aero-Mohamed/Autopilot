function [Mass, g, I, invI, timeSpan, dt, ICs, ICs_dot0, Vt0, ...
    dc, SD_Long, SD_Lat,SD_Lat_dash, initialGravity] = Input(inputs_filename)
    % Inputs 
    % here B2:B61 means read the excel sheet from cell B2 to cell B61
    aircraft_data=xlsread(inputs_filename,'B2:B61');
    
    % Integration time span & Step
    dt = aircraft_data(1);    
    tfinal = aircraft_data(2); 
    timeSpan = [0 tfinal];

    % Initial Conditions
    % [u; v; w; p; q; r; phi; theta; epsi; xe0; ye0; ze0]
    % ICs = [10; 2; 0; 2*pi/180; pi/180; 0; 20*pi/180; 15*pi/180; 30*pi/180; 2; 4; 7];
    ICs = aircraft_data(4:15);
    ICs_dot0 = zeros(12,1);
    Vt0 = sqrt(ICs(1)^2 + ICs(2)^2 + ICs(3)^2);    % Vto
    
    % control actions values
    % D_a, D_r, D_e, D_th
    dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];
    
    % gravity, mass % inertia
    Mass = aircraft_data(51);
    g = aircraft_data(52);
    Ixx = aircraft_data(53);
    Iyy = aircraft_data(54);
    Izz = aircraft_data(55);
    Ixz = aircraft_data(56);    
    Ixy=0;  Iyz=0;
    I = [Ixx , -Ixy , -Ixz ;...
        -Ixy , Iyy , -Iyz ;...
        -Ixz , -Iyz , Izz];
    invI = inv(I);
    
    
    % Stability Derivatives Longitudinal motion
    SD_Long = aircraft_data(21:36);
    
    % Stability Derivatives Lateral motion
    SD_Lat_dash = aircraft_data(37:50);
    SD_Lat_dash(9) = SD_Lat_dash(9)*Vt0;    % From dimension-less to dimensional
    SD_Lat_dash(10) = SD_Lat_dash(10)*Vt0;  % Form dimension-less to dimensional
    
    SD_Lat = LateralSD2BodyAxes(SD_Lat_dash, I);
    
    % initial gravity force
    [S, C, ~] = SCT(ICs(7:9));
    initialGravity = Mass*g*[
        S.theta; 
        -S.phi*C.theta;
        -C.phi*C.theta;
    ];
       
end

