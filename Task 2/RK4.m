% Function Represent the Runge-Kutta-4 Algorithm.
% returns the K1, K2, K3, K4 according to the input ICs.
% with time step of dt.
function K = RK4(ICs, dt)
    K = zeros(12, 4);
    K(:, 1) = dt*sixDOF(0, ICs );
    K(:, 2) = dt*sixDOF(0, ICs+0.5*K(:, 1) );
    K(:, 3) = dt*sixDOF(0, ICs+0.5*K(:, 2) );
    K(:, 4) = dt*sixDOF(0, ICs+K(:, 3) );
end