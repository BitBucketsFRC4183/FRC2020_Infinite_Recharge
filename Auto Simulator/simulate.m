physicsConstants;

state = [
    1; % x
    5; % y
    -pi-0.01; % heading, theta
    0.01;
    0.01;
];



u = [0; 0;];
T=0.02;

i = 0;
xs = zeros(1, 15/T + 1);
ys = zeros(1, 15/T + 1);
us = zeros(2, 15/T);

xs2 = zeros(1, 15/T + 1);
ys2 = zeros(1, 15/T + 1);



Q = [
    1/0.01^2, 0, 0, 0, 0;
    0, 1/0.01^2, 0, 0, 0;
    0, 0, 1/(1*pi/180)^2, 0, 0;
    0, 0, 0, 1/(0.1)^2, 0;
    0, 0, 0, 0, 1/(0.1)^2;
];

R = [
    1/12^2, 0;
    0, 1/12^2;
];

state0 = [0; 0; 0; 0; 0;];

states = zeros(5, 15/T + 1);

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
    
    A = sysd.A;
    B = sysd.B;
    
    P = dare(A, B, Q, R);
    K = (R+B'*P*B)\B'*P'*A;
    
    u = -K*(state - state0) - pinv(sysd.B)*(sysdF.B + A*state0 - state0);
    
    if (abs(u(1)) > 12)
        u(1) = sign(u(1))*12;
    end
    
    if (abs(u(2)) > 12)
        u(2) = sign(u(2))*12;
    end
    
    us(1:2, i) = u;
    
    state = A*state + B*u + sysdF.B;
    states(1:5, i) = state;
end

FIELD_W = (26 + 11.25/12)*0.3048;
FIELD_H = (52 + 5.25/12)*0.3048;

figure(1);
plot(xs, ys);
axis([-FIELD_W/2, FIELD_W/2, -FIELD_H/2, FIELD_H/2]);
daspect([1, 1, 1]);

figure(2);
plot(xs2, ys2);
axis([-FIELD_W/2, FIELD_W/2, -FIELD_H/2, FIELD_H/2]);
daspect([1, 1, 1]);

figure(3);
plot(0:T:15, us);

figure(4);
plot(0:T:15, states(4:5, 1:(15/T + 1)));