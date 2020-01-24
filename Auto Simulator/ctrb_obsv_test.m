W = 0.3048*(26 + (11.25)/12);
H = 0.3048*(52 + (5.25)/12);
V_MAX = 4.8768;

nC = 0;
nO = 0;
n = 10000;

C = eye(3);

for i = 1:n
    x = (2*rand - 1) * W;
    y = (2*rand - 1) * H;
    theta = (2*rand-1)*2*pi;
    omegaL = (2*rand-1)*V_MAX;
    omegaR = (2*rand-1)*V_MAX;
    
    u = [omegaL; omegaL; omegaR; omegaR];
    
    r = sqrt(x^2 + y^2);
    theta_r = atan2(y, x);
    
    state = [r; theta_r; theta;];
    
    Ac = getSysMatControl(state, u);
    Bc = getInpMatControl(state);
    
    if (rank(ctrb(Ac, Bc)) == 3)
        nC = nC + 1;
    end
    
    if (rank(obsv(Ac, C)) == 3)
        nO = nO + 1;
    end
end

disp("Controllable: " + (nC / n * 100) + "%");
disp("Observable: " + (nO / n * 100) + "%");