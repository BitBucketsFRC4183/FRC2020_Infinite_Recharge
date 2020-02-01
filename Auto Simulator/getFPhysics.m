function F = getFPhysics(state, A)
    physicsConstants;
    
    F = zeros(STATE_SIZE, 1);
    
    v0 = (state(vL) + state(vR))/2;
    
    F(X) = v0 * state(THETA) * cos(state(THETA));
    F(Y) = -v0 * state(THETA) * sin(state(THETA));
end