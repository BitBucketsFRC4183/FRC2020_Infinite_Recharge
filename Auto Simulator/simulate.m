physicsConstants;

state = [
    1; % x
    5; % y
    0.1; % heading, theta
    1;
    1;
];



u = [0; 0;];
T=0.02;

i = 0;
xs = zeros(1, 15/T + 1);
ys = zeros(1, 15/T + 1);
us = zeros(2, 15/T);



Q = [
    1, 0, 0, 0, 0;
    0, 1, 0, 0, 0;
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 1;
];

R = [
    1, 0;
    0, 1;
];

state0 = [0; 0; -pi/4; 0; 0;];

for t=0:T:15
    i = i + 1;
    
    xs(i) = state(X);
    ys(i) = state(Y);
    
    Ac = getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    Fc = getFPhysics(state);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    sysdF = c2d(syscF, T);
    
    K = place(sysd.A, sysd.B, [0.975, 0.976, 0.977, 0.978, 0.979]);
    u = -K*(state - state0) - pinv(sysd.B)*(sysdF.B + sysd.A*state0 - state0);
    
    if (abs(u(1)) > 12)
        u(1) = sign(u(1))*12;
    end
    
    if (abs(u(2)) > 12)
        u(2) = sign(u(2))*12;
    end
    
    us(1:2, i) = u;
    
    state = sysd.A * state + sysd.B * u + sysdF.B;
end

FIELD_W = (26 + 11.25/12)*0.3048;
FIELD_H = (52 + 5.25/12)*0.3048;

figure(1);
plot(xs, ys);
axis([-FIELD_W/2, FIELD_W/2, -FIELD_H/2, FIELD_H/2]);
daspect([1, 1, 1]);

figure(2);
plot(0:T:15, us);

lqr(sysd, Q, R);