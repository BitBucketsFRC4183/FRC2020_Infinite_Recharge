function A = getSysMatPhysics(state)
    physicsConstants;
    
    A = zeros(STATE_SIZE, STATE_SIZE);
    
    v0 = (state(vL) + state(vR))/2;
    
    
    
    A(X, THETA) = -v0*sin(state(THETA));
    A(X, vL) = cos(state(THETA))/2;
    A(X, vR) = cos(state(THETA))/2;
    
    A(Y, THETA) = v0*cos(state(THETA));
    A(Y, vL) = sin(state(THETA))/2;
    A(Y, vR) = sin(state(THETA))/2;
    
    A(THETA, vL) = -1/(2*rb);
    A(THETA, vR) = 1/(2*rb);
    
    A(vL, vL) = Am*C1;
    A(vL, vR) = Bm*C1;
    
    A(vR, vL) = Bm*C1;
    A(vR, vR) = Am*C1;
end