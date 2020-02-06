function x2 = badUpdate(x, T)
    physicsConstants;
    
    v = (x(vL) + x(vR))/2;
    theta = x(THETA);
    omega = (x(vR) - x(vL))/(2*rb);
    
    x2 = [
        x(X) + cos(theta)*v*T;
        x(Y) + sin(theta)*v*T;
        x(THETA) + omega*T;
        x(vL);
        x(vR);
    ];
end

