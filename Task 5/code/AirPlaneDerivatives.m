classdef AirPlaneDerivatives < handle 
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Longtudinal
        XU, ZU, MU, XW, ZW, MW, ZWD, ZQ, MWD, MQ, XDE, ZDE, MDE, XD_TH, ZD_TH, MD_TH
        % Lateral
        YV
        YB
        LBd, NBd, LPd, NPd, LRd, NRd, LDAd, LDRd,NDAd, NDRd
        LB, NB, LP, NP, LR, NR, YDA, YDR, LDA, NDA, LDR, NDR        
    end
    
    methods
        function obj = AirPlaneDerivatives(SD_Lat_dash , SD_Long, Inertia, ICs, g)
            
            [obj.YV, obj.YB, obj.LBd, obj.NBd, obj.LPd, obj.NPd, ...
                obj.LRd, obj.NRd, obj.YDA, obj.YDR, obj.LDAd, ...
                obj.NDAd, obj.LDRd, obj.NDRd] = feval(@(x) x{:}, num2cell(SD_Lat_dash));
            
            [obj.XU, obj.ZU, obj.MU, obj.XW, obj.ZW, obj.MW, obj.ZWD,...
                obj.ZQ, obj.MWD, obj.MQ, obj.XDE, obj.ZDE, obj.MDE, obj.XD_TH,...
                obj.ZD_TH, obj.MD_TH] = feval(@(x) x{:}, num2cell(SD_Long));
            
            LateralSD2BodyAxes(obj, Inertia);
        end
        
        function [obj] = LateralSD2BodyAxes(obj, Inertia)
            Ixx = Inertia(1);
            Izz = Inertia(9);
            Ixz = -Inertia(3);
            G = 1/(1 - Ixz^2 / Ixx / Izz);
            syms LB_ LP_ LR_ LDR_ LDA_ NB_ NP_ NR_ NDR_ NDA_
            eq1 = (LB_+Ixz*NB_/Ixx)*G == obj.LBd;
            eq2 = (NB_+Ixz*LB_/Izz)*G == obj.NBd;
            eq3 = (LP_+Ixz*NP_/Ixx)*G == obj.LPd;
            eq4 = (NP_+Ixz*LP_/Izz)*G == obj.NPd;
            eq5 = (LR_+Ixz*NR_/Ixx)*G == obj.LRd;
            eq6 = (NR_+Ixz*LR_/Izz)*G == obj.NRd;
            eq7 = (LDR_+Ixz*NDR_/Ixx)*G == obj.LDRd;
            eq8 = (NDR_+Ixz*LDR_/Izz)*G == obj.NDRd;
            eq9 = (LDA_+Ixz*NDA_/Ixx)*G == obj.LDAd;
            eq10 = (NDA_+Ixz*LDA_/Izz)*G == obj.NDAd;
            
            [A,B] = equationsToMatrix(...
            [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10],...
            [LB_ LP_ LR_ LDR_ LDA_ NB_ NP_ NR_ NDR_ NDA_]);
        
            X = A\B;
            X = vpa(X);
            
            obj.LB = X(1); 
            obj.LP = X(2) ;
            obj.LR = X(3) ;
            obj.LDR = X(4);
            obj.LDA = X(5);
            obj.NB = X(6);
            obj.NP = X(7);
            obj.NR = X(8);
            obj.NDR = X(9);
            obj.NDA = X(10);
            
        end
        
        
        
        function [A, B, C, D] = fullLinearModel(obj, ICs, g)
            
            u0 = ICs(1);
            w0 = ICs(3);
            theta0 = ICs(8);
            
            A =[obj.XU obj.XW -w0 -g*cos(theta0)
                obj.ZU/(1-obj.ZWD) obj.ZW/(1-obj.ZWD) (obj.ZQ+u0)/(1-obj.ZWD) -g*sin(theta0)/(1-obj.ZWD)
                obj.MU+obj.MWD*obj.ZU/(1-obj.ZWD) obj.MW+obj.MWD*obj.ZW/(1-obj.ZWD) obj.MQ+obj.MWD*(obj.ZQ+u0)/(1-obj.ZWD) -obj.MWD*g*sin(theta0)/(1-obj.ZWD)
                0 0 1 0];
            B = [obj.XDE obj.XD_TH;
                obj.ZDE/(1-obj.ZWD) obj.ZD_TH/(1-obj.ZWD);
                obj.MDE+obj.MWD*obj.ZDE/(1-obj.ZWD) obj.MD_TH+obj.MWD*obj.ZD_TH/(1-obj.ZWD);
                0 0];
            C = eye(4); 
            D = zeros(4,2);
            
        end
        
        function [A, B, C, D] = lateralFullLinearModel(obj, ICs, g)
            
            u0 = ICs(1);
            v0 = ICs(2);
            w0 = ICs(3);
            theta0 = ICs(8);
            
            Vto = sqrt(u0^2 + v0^2 + w0^2);
            Yp = 0;
            Yr = 0;
            
            A = [obj.YB/Vto (Yp+w0)/Vto (Yr-u0)/Vto g*cos(theta0)/Vto 0;...
                   obj.LBd obj.LPd obj.LRd 0 0;...
                   obj.NBd obj.NPd obj.NRd 0 0;...
                   0 1 tan(theta0) 0 0;...
                   0 0 1/cos(theta0) 0 0];
            B = [obj.YDA obj.YDR;...
               obj.LDAd obj.LDRd;...
               obj.NDAd obj.NDRd;...
               0 0;0 0];
            C = eye(5); D = zeros(5,2); 
            
        end
        
        
        function [A, B, C, D] = longPeriodModel(obj,ICs, g)
            u0 = ICs(1);
            
            A =[obj.XU -g
                -obj.ZU/(u0+obj.ZQ) 0];
            B =[obj.XDE obj.XD_TH
                -obj.ZDE/(obj.ZQ+u0) -obj.ZD_TH/(obj.ZQ+u0)];
            C = eye(2);
            D = zeros(2,2);
            
        end
        
    end
end

