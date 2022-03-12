% RBD Solver is function that implements Runge-Kutta-4
% Algorithm in order to integrate 6 DOF set of equations
% with a give Inital Conditions ICs, for single dt time step.
function state = RBDSolver(ICs, dt, Force, Moments, Mass, I, invI, g)
    
    K = zeros(12, 4);
    
    K(:, 1) = dt*DOF6(ICs ,Force, Moments, Mass, I, invI, g);
    K(:, 2) = dt*DOF6(ICs+0.5*K(:, 1) ,Force, Moments, Mass, I, invI, g);
    K(:, 3) = dt*DOF6(ICs+0.5*K(:, 2) ,Force, Moments, Mass, I, invI, g);
    K(:, 4) = dt*DOF6(ICs+K(:, 3) ,Force, Moments, Mass, I, invI, g);
    
    state = ICs + (...
        K(:, 1)+...
        2*K(:, 2)+...
        2*K(:, 3)+...
        K(:, 4))/6;
    
end