physicsConstants;

state = [
    10; % x
    5; % y
    3*pi/2; % heading, theta
    0; % omega of left front
    0; % omega of left back
    0; % omega of right front
    0; % omega of right back
];



u = [0; 0; 0; 0];
T=0.05;

i = 0;
xs = zeros(1, 15/T + 1);
ys = zeros(1, 15/T + 1);

ws = zeros(4, 15/T + 1);



Q = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 1;
];

R = [1; 1; 1; 1;];

for t=0:T:10*T
    i = i + 1;
    
    xs(i) = state(X);
    ys(i) = state(Y);
    
    ws(1:4, i) = state(OMEGA_LT:OMEGA_RB);
    
    Ac = getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    Fc = getFPhysics(state, Ac);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    sysdF = c2d(syscF, T);
    
    %K = lqr(sysdControl, Q, R);
    %u = K*state;
    
    state = sysd.A * state + sysd.B * u + sysdF.B;
end

FIELD_W = (26 + 11.25/12)*0.3048;
FIELD_H = (52 + 5.25/12)*0.3048;

figure(1);
plot(xs, ys);
axis([-FIELD_W/2, FIELD_W/2, -FIELD_H/2, FIELD_H/2]);
daspect([1, 1, 1]);

figure(2);
plot(0:T:15, ws);

c2d