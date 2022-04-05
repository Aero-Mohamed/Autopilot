classdef AirPlane < handle 
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       Mass
       g
       I            % Inirtia
       invI         % Inverse of Inirtia
       timeSpan
       dt
       ICs
       ICs_dot0
       Vt0
       dControl
       SD_Long
       SD_Lat
       SD_Lat_dash
       initialGravity
       airPlaneDerivatives      % Class
       rigidBodySolver          % Class
       
       u0, v0, w0, theta0, z0
       
    end
    
    methods
        function airPlane = AirPlane(inputsFilePath)
            % Inputs 
            % here B2:B61 means read the excel sheet from cell B2 to cell B61
            aircraft_data = xlsread(inputsFilePath,'B2:B61');
            % Integration time span & Step
            airPlane.dt = aircraft_data(1);    
            tfinal = aircraft_data(2); 
            airPlane.timeSpan = [0 tfinal];
            
            % Initial Conditions
            % [u; v; w; p; q; r; phi; theta; epsi; xe0; ye0; ze0]
            % ICs = [10; 2; 0; 2*pi/180; pi/180; 0; 20*pi/180; 15*pi/180; 30*pi/180; 2; 4; 7];
            airPlane.ICs = aircraft_data(4:15);
            airPlane.ICs_dot0 = zeros(12,1);
            airPlane.Vt0 = sqrt(airPlane.ICs(1)^2 + airPlane.ICs(2)^2 + airPlane.ICs(3)^2);    % Vto
            
            
            % D_a, D_r, D_e, D_th
            airPlane.dControl = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];
            
            % gravity, mass % inertia
            airPlane.Mass = aircraft_data(51);
            airPlane.g = aircraft_data(52);
            Ixx = aircraft_data(53);
            Iyy = aircraft_data(54);
            Izz = aircraft_data(55);
            Ixz = aircraft_data(56);    
            Ixy=0;  Iyz=0;
            airPlane.I = [Ixx , -Ixy , -Ixz ;...
                -Ixy , Iyy , -Iyz ;...
                -Ixz , -Iyz , Izz];
            airPlane.invI = inv(airPlane.I);
            
            % Stability Derivatives Longitudinal motion
            airPlane.SD_Long = aircraft_data(21:36);
            
            % Stability Derivatives Lateral motion
            airPlane.SD_Lat_dash = aircraft_data(37:50);
            airPlane.SD_Lat_dash(9) = airPlane.SD_Lat_dash(9)*airPlane.Vt0;    % From dimension-less to dimensional
            airPlane.SD_Lat_dash(10) = airPlane.SD_Lat_dash(10)*airPlane.Vt0;  % Form dimension-less to dimensional
            
            
            
            airPlane.airPlaneDerivatives = AirPlaneDerivatives(...
                airPlane.SD_Lat_dash , airPlane.SD_Long, airPlane.I);
            
            airPlane.rigidBodySolver = RigidBodySolver(airPlane.Mass, airPlane.I, airPlane.invI, airPlane.dt, airPlane.g);
            
            [S, C, ~] = SCT(airPlane.ICs(7:9));
            airPlane.initialGravity = airPlane.Mass*airPlane.g*[
                S.theta; 
                -S.phi*C.theta;
                -C.phi*C.theta;
            ];
        
            airPlane.u0 = airPlane.ICs(1);
            airPlane.v0 = airPlane.ICs(2);
            airPlane.w0 = airPlane.ICs(3);
            airPlane.theta0 = airPlane.ICs(8);
            airPlane.z0 = airPlane.ICs(12);
            
        end
        
        function [dForce, dMoment] = airFrame1(obj, state, forces, moments, dControl)

            [Da, Dr, De, Dth] = feval(@(x) x{:}, num2cell(dControl));

            Ixx = obj.I(1,1);
            Iyy = obj.I(2,2);
            Izz = obj.I(3,3);

            state_dot = obj.rigidBodySolver.DOF6(state, forces, moments);

            ds = state - obj.ICs;
            ds_dot = state_dot - obj.ICs_dot0;

            beta0 = asin(obj.ICs(2)/obj.Vt0);
            beta = asin(state(2)/obj.Vt0);
            dbeta = beta-beta0;

            dX = obj.Mass*(obj.airPlaneDerivatives.XU*ds(1)+ ...
                    obj.airPlaneDerivatives.XW*ds(3)+ ...
                    obj.airPlaneDerivatives.XDE*De+ ...
                    obj.airPlaneDerivatives.XD_TH*Dth);
                
            dY = obj.Mass*(obj.airPlaneDerivatives.YV*ds(2)+ ...
                obj.airPlaneDerivatives.YB*dbeta + ...
                obj.airPlaneDerivatives.YDA*Da + ...
                obj.airPlaneDerivatives.YDR*Dr);
            
            dZ = obj.Mass*(obj.airPlaneDerivatives.ZU*ds(1) + ...
                obj.airPlaneDerivatives.ZW*ds(3) + ...
                obj.airPlaneDerivatives.ZWD*ds_dot(3) + ...
                obj.airPlaneDerivatives.ZQ*ds(5) + ...
                obj.airPlaneDerivatives.ZDE*De + ...
                obj.airPlaneDerivatives.ZD_TH*Dth);

            dL = Ixx*(obj.airPlaneDerivatives.LB*dbeta + ...
                obj.airPlaneDerivatives.LP*ds(4) + ...
                obj.airPlaneDerivatives.LR*ds(6) + ...
                obj.airPlaneDerivatives.LDR*Dr + ...
                obj.airPlaneDerivatives.LDA*Da);
            
            dM = Iyy*(obj.airPlaneDerivatives.MU*ds(1) + ...
                obj.airPlaneDerivatives.MW*ds(3) + ...
                obj.airPlaneDerivatives.MWD*ds_dot(3) + ...
                obj.airPlaneDerivatives.MQ*ds(5) + ...
                obj.airPlaneDerivatives.MDE*De+ ...
                obj.airPlaneDerivatives.MD_TH*Dth);
            
            dN = Izz*(obj.airPlaneDerivatives.NB*dbeta + ...
                obj.airPlaneDerivatives.NP*ds(4) + ...
                obj.airPlaneDerivatives.NR*ds(6) + ...
                obj.airPlaneDerivatives.NDR*Dr + ...
                obj.airPlaneDerivatives.NDA*Da);

            dForce = [dX dY dZ];
            dMoment = [dL dM dN];

        end
        
        
        function [A_long, B_long, C_long, D_long] = fullLinearModel(obj)
            [A_long, B_long, C_long, D_long] = obj.airPlaneDerivatives.fullLinearModel(obj.ICs, obj.g);
        end
        
        function [A_phug, B_phug, C_phug, D_phug] = longPeriodModel(obj)
            [A_phug, B_phug, C_phug, D_phug] = obj.airPlaneDerivatives.longPeriodModel(obj.ICs, obj.g);
        end
        
    end
end

