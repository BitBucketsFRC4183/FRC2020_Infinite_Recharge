W = 0.3048*(26 + (11.25)/12);
H = 0.3048*(52 + (5.25)/12);
V_MAX = 4.8768;

nC = 0;
nO = 0;
n = 1000;

for i = 1:n
    x = (2*rand - 1) * W;
    y = (2*rand - 1) * H;
    theta = (2*rand-1)*2*pi;
    vl = (2*rand-1)*V_MAX;
    vr = (2*rand-1)*V_MAX;
    
    d = sqrt(x^2 + y^2);
    sq = sqrt(1 + x^2 / y^2);
    
    state = [x; y; theta; vl; vr;];
    
    Ac = getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    
    if (rank(ctrb(Ac, Bc)) == 5)
        nC = nC + 1;
    end
    
    C = [
        1/d, 1/d, 0, 0, 0;
        1/(y*sq), -x/(y^2*sq), -1, 0, 0;
        0, 0, 0, 1, 0;
        0, 0, 0, 0, 1;
    ];
    
    if (rank(obsv(Ac, C)) == 5)
        nO = nO + 1;
    end
end

disp("Controllable: " + (nC / n * 100) + "%");
disp("Observable: " + (nO / n * 100) + "%");