function B = getInpMatControl(state)
    controlConstants;
    
    B = zeros(STATE_SIZE, INPUT_SIZE);
    
    B(V, F_LT) = 1/M;
    B(V, F_LB) = 1/M;
    B(V, F_RT) = 1/M;
    B(V, F_RB) = 1/M;
    
    B(OMEGA, F_LT) = ROBOT_RADIUS / I_ROBOT;
    B(OMEGA, F_LB) = ROBOT_RADIUS / I_ROBOT;
    B(OMEGA, F_RT) = -ROBOT_RADIUS / I_ROBOT;
    B(OMEGA, F_RB) = -ROBOT_RADIUS / I_ROBOT;
    
    %B(OMEGA_L, VOLTS_L) = W_V;
    %B(OMEGA_LB, VOLTS_LB) = W_V;
    %B(OMEGA_R, VOLTS_R) = W_V;
    %B(OMEGA_RB, VOLTS_RB) = W_V;
end