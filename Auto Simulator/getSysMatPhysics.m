function A = getSysMatPhysics(state)
    physicsConstants;
    
    A = zeros(STATE_SIZE, STATE_SIZE);
    
    v = WHEEL_RADIUS/4*(state(OMEGA_LT) + state(OMEGA_LB) + state(OMEGA_RT) + state(OMEGA_RB));
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    A(X, OMEGA_LT) = cos(state(THETA)) * WHEEL_RADIUS/4;
    A(X, OMEGA_LB) = cos(state(THETA)) * WHEEL_RADIUS/4;
    A(X, OMEGA_RT) = cos(state(THETA)) * WHEEL_RADIUS/4;
    A(X, OMEGA_RB) = cos(state(THETA)) * WHEEL_RADIUS/4;
    A(X, THETA) = -v * sin(state(THETA));
    
    %
    % y' = v sin theta
    %
    % y'_v = sin theta0
    % y'_theta = v0 cos(theta0)
    %
    A(Y, OMEGA_LT) = sin(state(THETA)) * WHEEL_RADIUS/4;
    A(Y, OMEGA_LB) = sin(state(THETA)) * WHEEL_RADIUS/4;
    A(Y, OMEGA_RT) = sin(state(THETA)) * WHEEL_RADIUS/4;
    A(Y, OMEGA_RB) = sin(state(THETA)) * WHEEL_RADIUS/4;
    A(Y, THETA) = v * cos(state(THETA));
    
    A(THETA, OMEGA_LT) = -WHEEL_RADIUS/2 / ROBOT_RADIUS;
    A(THETA, OMEGA_LB) = -WHEEL_RADIUS/2 / ROBOT_RADIUS;
    A(THETA, OMEGA_RT) = WHEEL_RADIUS/2 / ROBOT_RADIUS;
    A(THETA, OMEGA_RB) = WHEEL_RADIUS/2 / ROBOT_RADIUS;
    
    A(OMEGA_LT, OMEGA_LT) = W_W;
    A(OMEGA_LB, OMEGA_LB) = W_W;
    A(OMEGA_RT, OMEGA_RT) = W_W;
    A(OMEGA_RB, OMEGA_RB) = W_W;
end