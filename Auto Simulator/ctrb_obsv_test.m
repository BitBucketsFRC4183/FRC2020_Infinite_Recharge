physicsConstants;

W = 0.3048*(26 + (11.25)/12);
H = 0.3048*(52 + (5.25)/12);
V_MAX = 4.8768;

nC = 0;
nO = 0;
n = 1000;

xT = 0.3048*5.58;
yT = 15.9830/2;

thetaLL = pi/6;
rLL = 0.2286;



for i = 1:n
    x = (2*rand - 1) * W;
    y = (2*rand - 1) * H;
    thetaR = (2*rand-1)*2*pi;
    vl = (2*rand-1)*V_MAX;
    vr = (2*rand-1)*V_MAX;
    
    d = sqrt(x^2 + y^2);
    sq = sqrt(1 + x^2 / y^2);
    
    state = [x; y; thetaR; vl; vr;];
    
    Ac = getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    C = getCPhysics(state);
    
    if (rank(ctrb(Ac, Bc)) == STATE_SIZE)
        nC = nC + 1;
    end
    
    if (rank(obsv(Ac, C)) == STATE_SIZE)
        nO = nO + 1;
    end
end

disp("Controllable: " + (nC / n * 100) + "%");
disp("Observable: " + (nO / n * 100) + "%");