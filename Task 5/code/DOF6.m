% Function return the set of 12 state of the six degree of freedom
% system after sub., with given ICs
function F = DOF6(ICs, forces, Moments, Mass, Inertia, invI, g)

    % (Sin, Cos, Tan) of (phi, theta, epsi)
    [S, C, T] = SCT(ICs(7:9));
    s_theta = S.theta;
    c_theta = C.theta;
    t_theta = T.theta;
    s_epsi = S.epsi;
    c_epsi = C.epsi;
    s_phi = S.phi;
    c_phi = C.phi;
    
    Forces = forces + Mass*g*[ 
        -s_theta; 
        s_phi*c_theta;
        c_phi*c_theta;
    ];

    % (u, v, w) dot
    u_v_w_dot = (1/Mass)*Forces - cross(...
        ICs(4:6, 1), ICs(1:3, 1)...
    );

    % (p, q, r) dot
    p_q_r_dot = invI *(Moments - cross(...
        ICs(4:6, 1), Inertia * ICs(4:6, 1)...
    ));
    
    % (phi, theta, epsi) dot
    phi_theta_epsi_dot = [
        1, s_phi*t_theta, c_phi*t_theta;
        0, c_phi, -s_phi;
        0, s_phi/c_theta, c_phi/c_theta;
    ] * ICs(4:6, 1);

    % (x, y, z) dot
    x_y_z_dot = [
        c_theta*c_epsi, (s_phi*s_theta*c_epsi - c_phi*s_epsi), (c_phi*s_theta*c_epsi + s_phi*s_epsi);
        c_theta*s_epsi, (s_phi*s_theta*s_epsi + c_phi*c_epsi), (c_phi*s_theta*s_epsi - s_phi*c_epsi);
        -s_theta, s_phi*c_theta, c_phi*c_theta
    ] * ICs(1:3, 1);

    F = [u_v_w_dot; p_q_r_dot; phi_theta_epsi_dot; x_y_z_dot];

end