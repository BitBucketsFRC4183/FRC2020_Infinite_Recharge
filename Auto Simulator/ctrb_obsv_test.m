W = 26 + (11.25)/12;
H = (52 + (5.25)/12);
V_MAX = 16;

nC = 0;
nO = 0;
n = 10000;

C = eye(9);

for i = 1:n
    x = (2*rand - 1) * W;
    y = (2*rand - 1) * H;
    v = (2*rand - 1) * V_MAX;
    theta = 2*pi*rand;
    omega = 2*pi*rand;
    
    r = sqrt(x^2 + y^2);
    theta_r = atan2(y, x);
    
    state = [r; theta_r; v; theta; omega];
    
    Ac = getSysMatControl(state);
    Bc = getInpMatControl(state);
    
    if (rank(ctrb(Ac, Bc)) == 9)
        nC = nC + 1;
    end
    
    if (rank(obsv(Ac, C)) == 9)
        nO = nO + 1;
    end
end

disp("Controllable: " + (nC / n * 100) + "%");
disp("Observable: " + (nO / n * 100) + "%");