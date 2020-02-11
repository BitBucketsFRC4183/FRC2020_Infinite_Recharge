function C = getCPhysics(state)
    physicsConstants;
    
    C = zeros(OUTPUT_SIZE, STATE_SIZE);
    
    
    
    x = state(X);
    y = state(Y);
    thetaR = state(THETA);
    
    
    
    d = sqrt((x - xT)^2 + (y - yT)^2);
    num = yT - y - rLL*cos(thetaLL + thetaR);
    denom = xT - x - rLL*sin(thetaLL + thetaR);
    
    ratio = (yT - y)/(xT - x);
    
    
    C(D, X) = (x - xT)/d;%num/denom^2*1/sqrt(1+ratio^2);
    C(D, Y) = (y - yT)/d;%1/denom*1/sqrt(1+ratio^2);
    %C(D, THETA) = 0(1-(denom*rLL*sin(thetaLL+thetaR)+num*rLL*cos(thetaLL+thetaR))/(denom^2));
    
    C(TX, X) = -1/((x-xT)*(1 + ratio^2));
    C(TX, Y) = (y-yT)/((x-xT)^2 * (1 + ratio^2));
    C(TX, THETA) = 1;
    
    C(VL_O, vL) = 1;
    
    C(VR_O, vR) = 1;
    
    C(LL_THETA, THETA) = 1;
end
