function C = getCPhysics(state)
    physicsConstants;
    
    C = zeros(OUTPUT_SIZE, STATE_SIZE);
    
    
    
    x = state(X);
    y = state(Y);
    thetaR = state(THETA);
    
    
    
    d = sqrt((x - xT)^2 + (y - yT)^2);
    num = yT - y - rLL*cos(thetaLL + thetaR);
    denom = xT - x - rLL*sin(thetaLL + thetaR);
    
    ratio = (yT - y - rLL*cos(thetaLL + thetaR))/(xT - x - rLL*sin(thetaLL + thetaR));
    
    
    C(D, X) = num/denom^2*1/sqrt(1+ratio^2);
    C(D, Y) = 1/denom*1/sqrt(1+ratio^2);
    C(D, THETA) = (1-(denom*rLL*sin(thetaLL+thetaR)+num*rLL*cos(thetaLL+thetaR))/(denom^2));
    
    C(TX, X) = (x-xT)/d;
    C(TX, Y) = (y-yT)/d;
    C(TX, THETA) = (x-xT)/d*(-rLL*sin(thetaLL + thetaR))+(y-yT)/d*(rLL*cos(thetaLL+thetaR));
    
    C(VL_O, vL) = 1;
    
    C(VR_O, vR) = 1;
end
