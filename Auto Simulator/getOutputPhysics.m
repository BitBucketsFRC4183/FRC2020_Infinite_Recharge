function y = getOutputPhysics(state)
    physicsConstants;

    x = state(X);
    y = state(Y);
    thetaR = state(THETA);
    
    xLL = x;%
    yLL = y;%
    
    d = sqrt((xLL - xT)^2 + (yLL - yT)^2);% + 0.125*(2*rand-1);
    tx = thetaR - atan2(yT - yLL, xT - xLL - xT);% + pi/1800*(2*rand-1);
    
    y = [
        d;
        tx;
        state(vL);
        state(vR);
    ];
end

