function B = getInpMatPhysics(state)
    physicsConstants;
    
    B = zeros(STATE_SIZE, INPUT_SIZE);
    
    %
    % x' = v cos theta
    %
    % x'_v = cos theta0
    % x'_theta = -v0 sin(theta0)
    %
    
    %
    % y' = v sin theta
    %
    % y'_v = sin theta0
    % y'_theta = v0 cos(theta0)
    %
    
    B(V, VOLTS_LT) = F_V / M;
    B(V, VOLTS_LB) = F_V / M;
    B(V, VOLTS_RT) = F_V / M;
    B(V, VOLTS_RB) = F_V / M;
    
    B(OMEGA, VOLTS_LT) = ROBOT_RADIUS * F_V / I_ROBOT;
    B(OMEGA, VOLTS_LB) = ROBOT_RADIUS * F_V / I_ROBOT;
    B(OMEGA, VOLTS_RT) = -ROBOT_RADIUS * F_V / I_ROBOT;
    B(OMEGA, VOLTS_RB) = -ROBOT_RADIUS * F_V / I_ROBOT;
    
    B(OMEGA_LT, VOLTS_LT) = W_V;
    B(OMEGA_LB, VOLTS_LB) = W_V;
    B(OMEGA_RT, VOLTS_RT) = W_V;
    B(OMEGA_RB, VOLTS_RB) = W_V;
end