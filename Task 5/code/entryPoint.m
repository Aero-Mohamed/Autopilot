function [] = entryPoint()
    plane = AirPlane("NT-33A_4.xlsx");
    steps = (plane.timeSpan(2) - plane.timeSpan(1))/plane.dt;
    Result = NaN(12, steps);
    Result(:,1) = plane.ICs;
    time_V = linspace(0, plane.timeSpan(2), steps+1);
    
    dForces = [0 ; 0; 0];
    dMoments = [0 ; 0; 0];

    for i =1:steps
        Result(:, i+1) = plane.rigidBodySolver.nextStep( ...
            Result(:, i),(plane.initialGravity + dForces), dMoments ...
        );    

        [dF, dM] = plane.airFrame1(Result(:, i+1), ...
            (plane.initialGravity + dForces), dMoments, plane.dControl ...
        );

        dForces = vpa(dF');
        dMoments = vpa(dM');
    end
    
    theta = Result(8,:);
    theta_deg=theta*180/pi;
    
    figure
    hold on
    plot(time_V, theta_deg, '-', 'DisplayName', '\Theta (Non-Linear)');                  % Non-Linear Model
    title('theta (deg/sec)'); xlabel('t (sec)');
    legend('show');
    grid on
end