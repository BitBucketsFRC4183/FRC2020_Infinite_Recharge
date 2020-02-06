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
    
    if (rank(ctrb(Ac, Bc)) == 5)
        nC = nC + 1;
    end
    
    %                     (yT - y - rLL cos(thetaLL + thetaR))
    % tx = thetaR - arctan(==================================)
    %                     (xT - x - rLL sin(thetaLL + thetaR))
    
    % tx_denom_x = -1
    % tx_num_y = -1
    num = yT - y - rLL*cos(thetaLL + thetaR);
    denom = xT - x - rLL*sin(thetaLL + thetaR);
    
    ratio = (yT - y - rLL*cos(thetaLL + thetaR))/(xT - x - rLL*sin(thetaLL + thetaR));
    % tx_x = num/denom^2 *1/sqrt(1+ratio^2)
    % tx_y = 1/denom *1/sqrt(1+ratio^2)
    % tx_theta = 1 - (denom*rLLsin(thetaLL+thetaR) + rLL*cos(thetaLL+thetaR))/(denom^2)
    
    
    C = [
        num/denom^2*1/sqrt(1+ratio^2), 1/denom*1/sqrt(1+ratio^2), (1-(denom*rLL*sin(thetaLL+thetaR)+num*rLL*cos(thetaLL+thetaR))/(denom^2)), 0, 0;
        (x-xT)/d, (y-yT)/d, (x-xT)/d*(-rLL*sin(thetaLL + thetaR))+(y-yT)/d*(rLL*cos(thetaLL+thetaR)), 0, 0;
        0, 0, 0, 1, 0;
        0, 0, 0, 0, 1;
    ];
    
    if (rank(obsv(Ac, C)) == 5)
        nO = nO + 1;
    end
end

disp("Controllable: " + (nC / n * 100) + "%");
disp("Observable: " + (nO / n * 100) + "%");