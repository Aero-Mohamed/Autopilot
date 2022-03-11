% Function return the set of 12 state of the six degree of freedom
% system after sub., with given ICs
function F = DOF6(~, ICs)
    [forces,Moments,Mass,Inertia,~,~,~] = Input();
    % (Sin, Cos, Tan) of (phi, theta, epsi)
    [S, C, T] = SCT(ICs(7:9));
    
    Forces = forces + Mass*9.81*[ 
        -S.theta; 
        S.phi*C.theta;
        C.phi*C.theta;
    ];

    % (u, v, w) dot
    F(1:3, 1) = Forces/Mass - cross(...
        ICs(4:6, 1), ICs(1:3, 1)...
    );

    % (p, q, r) dot
    F(4:6, 1) = Inertia\(Moments - cross(...
        ICs(4:6, 1), Inertia * ICs(4:6, 1)...
    ));
    
    % (phi, theta, epsi) dot
    F(7:9, 1) = [
        1, S.phi*T.theta, C.phi*T.theta;
        0, C.phi, -S.phi;
        0, S.phi/C.theta, C.phi/C.theta;
    ] * ICs(4:6, 1);

    % (x, y, z) dot
    F(10:12, 1) = [
        C.theta*C.epsi, (S.phi*S.theta*C.epsi - C.phi*S.epsi), (C.phi*S.theta*C.epsi + S.phi*S.epsi);
        C.theta*S.epsi, (S.phi*S.theta*S.epsi + C.phi*C.epsi), (C.phi*S.theta*S.epsi - S.phi*C.epsi);
        -S.theta, S.phi*C.theta, C.phi*C.theta
    ] * ICs(1:3, 1);

end