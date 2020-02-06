function A = getABad(state)
    physicsConstants;

    theta = state(THETA);

    A = [
        0, 0, 0, cos(theta)/2, cos(theta)/2;
        0, 0, 0, sin(theta)/2, sin(theta)/2;
        0, 0, 0, -1/(2*rb), 1/(2*rb);
        0, 0, 0, Am*C1, Bm*C1;
        0, 0, 0, Bm*C1, Am*C1;
    ];
end

