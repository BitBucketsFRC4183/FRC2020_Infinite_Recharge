function y = robotSystemUKF_output(x, u)
    physicsConstants;
    
    y = zeros(OUTPUT_SIZE, 1);
    
    xLL = x(X) + xc*cos(x(THETA)) - yc*sin(x(THETA)) + r*cos(x(THETA));
    yLL = y(X) + xc*sin(x(THETA)) + yc*cos(x(THETA)) + r*sin(x(THETA));
    
    y(D) = sqrt((xT - x(X))^2 + (yT - x(Y))^2);
    % tan TX = y/x
    y(TX) = atan2(yT - x(Y), xT - x(X)) - x(THETA);
    y(VL_O) = x(vL);
    y(VR_O) = x(vR);
    y(LL_THETA) = x(THETA);
    y(OMEGA) = (x(vR) - x(vL))/(2*rb);
end

