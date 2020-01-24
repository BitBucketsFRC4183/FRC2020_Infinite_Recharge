function B = getInpMatControl(state)
    controlConstants;
    
    B = zeros(STATE_SIZE, INPUT_SIZE);
    
    B(R, OMEGA_LT) = cos(state(THETA) - state(THETA_R)) * WHEEL_RADIUS / 4;
    B(R, OMEGA_LB) = cos(state(THETA) - state(THETA_R)) * WHEEL_RADIUS / 4;
    B(R, OMEGA_RT) = cos(state(THETA) - state(THETA_R)) * WHEEL_RADIUS / 4;
    B(R, OMEGA_RB) = cos(state(THETA) - state(THETA_R)) * WHEEL_RADIUS / 4;
    
    B(THETA_R, OMEGA_LT) = -sin(state(THETA) - state(THETA_R))/state(R) * WHEEL_RADIUS / 4;
    B(THETA_R, OMEGA_LB) = -sin(state(THETA) - state(THETA_R))/state(R) * WHEEL_RADIUS / 4;
    B(THETA_R, OMEGA_RT) = -sin(state(THETA) - state(THETA_R))/state(R) * WHEEL_RADIUS / 4;
    B(THETA_R, OMEGA_RB) = -sin(state(THETA) - state(THETA_R))/state(R) * WHEEL_RADIUS / 4;
    
    B(THETA, OMEGA_LT) = -WHEEL_RADIUS/2 / ROBOT_RADIUS;
    B(THETA, OMEGA_LB) = -WHEEL_RADIUS/2 / ROBOT_RADIUS;
    B(THETA, OMEGA_RT) = WHEEL_RADIUS/2 / ROBOT_RADIUS;
    B(THETA, OMEGA_RB) = WHEEL_RADIUS/2 / ROBOT_RADIUS;
    %B(V, F_LT) = 1/M;
    %B(V, F_LB) = 1/M;
    %B(V, F_RT) = 1/M;
    %B(V, F_RB) = 1/M;
    
    %B(OMEGA, F_LT) = ROBOT_RADIUS / I_ROBOT;
    %B(OMEGA, F_LB) = ROBOT_RADIUS / I_ROBOT;
    %B(OMEGA, F_RT) = -ROBOT_RADIUS / I_ROBOT;
    %B(OMEGA, F_RB) = -ROBOT_RADIUS / I_ROBOT;
    
    %B(OMEGA_L, VOLTS_L) = W_V;
    %B(OMEGA_LB, VOLTS_LB) = W_V;
    %B(OMEGA_R, VOLTS_R) = W_V;
    %B(OMEGA_RB, VOLTS_RB) = W_V;
end