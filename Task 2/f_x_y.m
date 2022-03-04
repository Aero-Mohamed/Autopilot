function states = f_x_y(IC_vec,F__,Moments,I,mass)
u0 = IC_vec(1);
v0 = IC_vec(2);
w0 = IC_vec(3);
p0 = IC_vec(4);
q0 = IC_vec(5);
r0 = IC_vec(6);
phi0 = IC_vec(7);
theta0 = IC_vec(8); 
epsi0 = IC_vec(9);
XE0 = IC_vec(10);
YE0 = IC_vec(11);
ZE0 = IC_vec(12);
Forces = F__+mass*9.81*[-sin(theta0);sin(phi0)*cos(theta0);cos(phi0)*cos(theta0)];


    states(1:3,1) = 1/mass*Forces-cross([p0;q0;r0],[u0;v0;w0]);
    states(4:6,1) = inv(I)*(Moments-cross([p0;q0;r0],I*[p0;q0;r0]));
    states(7:9,1) =[1 sin(phi0)*tan(theta0) cos(phi0)*tan(theta0);0 cos(phi0) -sin(phi0);0 sin(phi0)/cos(theta0) cos(phi0)/cos(theta0)]*[p0;q0;r0];
    states(10:12,1) = [cos(theta0)*cos(epsi0) sin(phi0)*sin(theta0)*cos(epsi0)-cos(phi0)*sin(epsi0) cos(phi0)*sin(theta0)*cos(epsi0)+sin(phi0)*sin(epsi0);
         cos(theta0)*sin(epsi0) sin(phi0)*sin(theta0)*sin(epsi0)+cos(phi0)*cos(epsi0) cos(phi0)*sin(theta0)*sin(epsi0)-sin(phi0)*cos(epsi0);
         -sin(theta0),sin(phi0)*cos(theta0),cos(phi0)*cos(theta0)]*[u0;v0;w0];
   
    
end

