function F = getFPhysics(state, A)
    physicsConstants;
    
    F = zeros(STATE_SIZE, 1);
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    % x' = v0 cos(theta0) + x'_v * (v - 0) - v0 sin(theta0) * (theta - theta0)
    %
    F(X) = state(V) * cos(state(THETA)) - A(1, 1:STATE_SIZE)*state;
    F(Y) = state(V) * sin(state(THETA)) - A(2, 1:STATE_SIZE)*state;
end