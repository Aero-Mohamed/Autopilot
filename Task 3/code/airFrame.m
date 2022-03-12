function [dForce, dMoment] = airFrame(SD_Long, SD_Lat, dControl, state0, state0_dot, state, Vt0 ,forces, moments, Mass, I, invI, g)
    
    [YV, YB, LB, NB, LP, NP, LR, NR, YDA, YDR, LDA, NDA, LDR, NDR] = feval(@(x) x{:}, num2cell(SD_Lat));
    [XU, ZU, MU, XW, ZW, MW, ZWD, ZQ, MWD, MQ, XDE, ZDE, MDE, XD_TH, ZD_TH, MD_TH] = feval(@(x) x{:}, num2cell(SD_Long));
    [Da, Dr, De, Dth] = feval(@(x) x{:}, num2cell(dControl));
    
    Ixx = I(1,1);
    Iyy = I(2,2);
    Izz = I(3,3);
    
    state_dot = DOF6(state, forces, moments, Mass, I, invI, g);
    
    ds = state - state0;
    ds_dot = state_dot - state0_dot;
    
    beta0 = asin(state0(2)/Vt0);
    beta = asin(state(2)/Vt0);
    dbeta = beta-beta0;
    
    dX = Mass*(XU*ds(1)+XW*ds(3)+XDE*De+XD_TH*Dth);
    dY = Mass*(YV*ds(2)+YB*dbeta+YDA*ds(1)+YDR*Dr);
    dZ = Mass*(ZU*ds(1)+ZW*ds(3)+ZWD*ds_dot(3)+ZQ*ds(5)+ZDE*De+ZD_TH*Dth);
    
    dL = Ixx*(LB*dbeta+LP*ds(4)+LR*ds(6)+LDR*Dr+LDA*Da);
    dM = Iyy*(MU*ds(1)+MW*ds(3)+MWD*ds_dot(3)+MQ*ds(5)+MDE*De+MD_TH*Dth);
    dN = Izz*(NB*dbeta+NP*ds(4)+NR*ds(6)+NDR*Dr+NDA*Da);
    
    dForce = [dX dY dZ];
    dMoment = [dL dM dN];
    
end