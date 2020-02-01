function A = getSysMatControl(state)
    controlConstants;
    
    A = zeros(STATE_SIZE, STATE_SIZE);
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    v = WHEEL_RADIUS / 4 * (u(OMEGA_LT) + u(OMEGA_LB) + u(OMEGA_RT) + u(OMEGA_RB));
    A(R, THETA) = v * cos(state(THETA) - state(THETA_R));
    A(R, THETA_R) = -v * cos(state(THETA) - state(THETA_R));
    %A(X, V) = cos(state(THETA));
    %A(X, THETA) = -state(V) * sin(state(THETA));
    
    %
    % y' = v sin theta
    %
    % y'_v = sin theta0
    % y'_theta = v0 cos(theta0)
    %
    %A(THETA_R, V) = sin(state(THETA) - state(THETA_R))/state(R);
    A(THETA_R, R) = -v/state(R)^2 * sin(state(THETA) - state(THETA_R));
    A(THETA_R, THETA) = v/state(R) * cos(state(THETA) - state(THETA_R));
    A(THETA_R, THETA_R) = -v/state(R) * cos(state(THETA) - state(THETA_R));
    %A(Y, V) = sin(state(THETA));
    %A(Y, THETA) = state(V) * cos(state(THETA));
    
    %A(THETA, OMEGA) = 1;
    
    %A(V, F_LT) = 1 / M;
    %A(V, F_LB) = 1 / M;
    %A(V, F_RT) = 1 / M;
    %A(V, F_RB) = 1 / M;
    
    %A(OMEGA, F_LT) = ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_LB) = ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_RT) = -ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_RB) = -ROBOT_RADIUS / I_ROBOT;
    
    %A(OMEGA_L, OMEGA_L) = W_W;
    %A(OMEGA_LB, OMEGA_LB) = W_W;
    %A(OMEGA_R, OMEGA_R) = W_W;
    %A(OMEGA_RB, OMEGA_RB) = W_W;
    
    %A(OMEGA, F_LT) = ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_LB) = ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_RT) = -ROBOT_RADIUS / I_ROBOT;
    %A(OMEGA, F_RB) = -ROBOT_RADIUS / I_ROBOT;
end