function B = getInpMatPhysics(state)
    physicsConstants;
    
    B = zeros(STATE_SIZE, INPUT_SIZE);
    
    B(vL, VL) = Am*C2;
    B(vL, VR) = Bm*C2;
    
    B(vR, VL) = Bm*C2;
    B(vR, VR) = Am*C2;
end