function y = robotSystemUKF_output(x, u)
    physicsConstants;
    
    y = zeros(OUTPUT_SIZE, 1);
    
    y(D) = sqrt(x(X)^2 + x(Y)^2);
    % assume LL is always facing to x+
    % tan TX = y/x
    y(TX) = atan2(-x(Y), -x(X));
    y(VL_O) = x(vL);
    y(VR_O) = x(vR);
    y(LL_THETA) = x(THETA);
    y(OMEGA) = (x(vR) - x(vL))/(2*rb);
end

