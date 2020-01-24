function A = getSysMatPhysics(state)
    physicsConstants;
    
    A = zeros(STATE_SIZE, STATE_SIZE);
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    A(X, V) = cos(state(THETA));
    A(X, THETA) = -state(V) * sin(state(THETA));
    
    %
    % y' = v sin theta
    %
    % y'_v = sin theta0
    % y'_theta = v0 cos(theta0)
    %
    A(Y, V) = sin(state(THETA));
    A(Y, THETA) = state(V) * cos(state(THETA));
    
    A(V, OMEGA_LT) = F_W / M;
    A(V, OMEGA_LB) = F_W / M;
    A(V, OMEGA_RT) = F_W / M;
    A(V, OMEGA_RB) = F_W / M;
    
    A(THETA, OMEGA) = 1;
    
    A(OMEGA, OMEGA_LT) = ROBOT_RADIUS * F_W / I_ROBOT;
    A(OMEGA, OMEGA_LB) = ROBOT_RADIUS * F_W / I_ROBOT;
    A(OMEGA, OMEGA_RT) = -ROBOT_RADIUS * F_W / I_ROBOT;
    A(OMEGA, OMEGA_RB) = -ROBOT_RADIUS * F_W / I_ROBOT;
    
    A(OMEGA_LT, OMEGA_LT) = W_W;
    A(OMEGA_LB, OMEGA_LB) = W_W;
    A(OMEGA_RT, OMEGA_RT) = W_W;
    A(OMEGA_RB, OMEGA_RB) = W_W;
end