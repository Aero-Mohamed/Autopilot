function SD_Lat = LateralSD2BodyAxes(SD_Lat_dash, I)
    YV      = SD_Lat_dash(1);
    YB      = SD_Lat_dash(2); 
    YDA     = SD_Lat_dash(9); 
    YDR     = SD_Lat_dash(10);
    
    LBd     = SD_Lat_dash(3); 
    NBd     = SD_Lat_dash(4); 
    LPd     = SD_Lat_dash(5);
    NPd     = SD_Lat_dash(6);
    LRd     = SD_Lat_dash(7);
    NRd     = SD_Lat_dash(8);
    LDAd    = SD_Lat_dash(11);
    NDAd    = SD_Lat_dash(12);
    LDRd    = SD_Lat_dash(13);
    NDRd    = SD_Lat_dash(14);
    
    Ixx = I(1);
    Izz = I(9);
    Ixz = I(3);
    
    G = 1/(1 - Ixz^2 / Ixx / Izz);
    
    syms LB LP LR LDR LDA NB NP NR NDR NDA
    eq1 = (LB+Ixz*NB/Ixx)*G == LBd;
    eq2 = (NB+Ixz*LB/Izz)*G == NBd;
    eq3 = (LP+Ixz*NP/Ixx)*G == LPd;
    eq4 = (NP+Ixz*LP/Izz)*G == NPd;
    eq5 = (LR+Ixz*NR/Ixx)*G == LRd;
    eq6 = (NR+Ixz*LR/Izz)*G == NRd;
    eq7 = (LDR+Ixz*NDR/Ixx)*G == LDRd;
    eq8 = (NDR+Ixz*LDR/Izz)*G == NDRd;
    eq9 = (LDA+Ixz*NDA/Ixx)*G == LDAd;
    eq10 = (NDA+Ixz*LDA/Izz)*G == NDAd;
    
    [A,B] = equationsToMatrix(...
        [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10],...
        [LB LP LR LDR LDA NB NP NR NDR NDA]);
    
    X = A\B;
    
    SD_Lat = [
        YV YB ...
        X(1) X(6) X(2) X(7) ...
        X(3) X(8) ...
        YDA YDR ...
        X(5) X(10) X(4) X(9) ...
    ]';
    SD_Lat = vpa(SD_Lat);
    
end