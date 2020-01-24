function F = getFPhysics(state, A)
    physicsConstants;
    
    F = zeros(STATE_SIZE, 1);
    
    v = WHEEL_RADIUS/4*(state(OMEGA_LT) + state(OMEGA_LB) + state(OMEGA_RT) + state(OMEGA_RB));
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    % x' = v0 cos(theta0) + x'_v * (v - 0) - v0 sin(theta0) * (theta - theta0)
    %
    F(X) = v * cos(state(THETA)) - A(1, 1:STATE_SIZE)*state;
    F(Y) = v * sin(state(THETA)) - A(2, 1:STATE_SIZE)*state;
end