physicsConstants;

state = [
    0.0; % x
    0.0; % y
    0.0; % heading, theta
    0;
    0;
];

state_hat = [5; 5; pi/3; 0; 0;];

P = 0.01*eye(5);

Q = [
    0.125, 0;
    0, 0.125;
];

R = [
    0.125, 0, 0, 0;
    0, pi/1800, 0, 0;
    0, 0, 0.0001, 0; % pretty certain about velocities
    0, 0, 0, 0.0001;
];



u = [7; 10;];
T=0.02;

i = 0;

TMAX = 1;

xs = zeros(1, TMAX/T + 1);
ys = zeros(1, TMAX/T + 1);
xs_hat = zeros(1, TMAX/T + 1);
ys_hat = zeros(1, TMAX/T + 1);
us = zeros(2, TMAX/T);

xs2 = zeros(1, TMAX/T + 1);
ys2 = zeros(1, TMAX/T + 1);

states = zeros(5, TMAX/T + 1);
states_hat = zeros(5, TMAX/T + 1);

es = zeros(1, TMAX/T + 1);


kP = 2;
kI = 0.0;
kD = 0;
kF = 2.551777109743181;

lastVLerr = 0;
lastVRerr = 0;

vLfinal = 3;
vRfinal = 5;



for t=0:T:TMAX
    i = i + 1;
    
    xs(i) = state(X);
    ys(i) = state(Y);
    
    
    
    Ac = getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    Fc = getFPhysics(state);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    %syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    %sysdF = c2d(syscF, T);
    
    A = sysd.A;
    B = sysd.B;
    F = zeros(5, 1);%sysdF.B;
    
    lerr = state(vL) - vLfinal;
    rerr = state(vR) - vRfinal;
    
    %u(1) = kF*vLfinal + kP*lerr;
    %u(2) = kF*vRfinal + kP*rerr;
    
    u(1) = 0;
    u(2) = 0;

    %u(1) + 0.05*(2*rand-1);
    %u(2) + 0.05*(2*rand-1);
    
    us(1:2, i) = u;
    
    % move state forward in time
    %state = badUpdate(state, T);
    state = A*state + B*u + F;
    %state(THETA) = mod(state(THETA), 2*pi);
    
    states(1:5, i) = state;
    
    
    
    Ac = getSysMatPhysics(state_hat);
    Bc = getInpMatPhysics(state_hat);
    Fc = getFPhysics(state_hat);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    sysdF = c2d(syscF, T);
    
    A = sysd.A;
    B = sysd.B;
    G = B;
    
    % predict
    state_hat = A*state_hat + B*u + sysdF.B;
    P = A*P*A' + G*Q*G';
    
    % measure
    y = getOutputPhysics(state);
    
    % update
    C = getCPhysics(state_hat);
    K = P*C'/(C*P*C'+R);
    state_hat = state_hat + K * (y - getOutputPhysics(state_hat)); 
    P = (eye(5) - K*C)*P;
    
    xs_hat(i) = state_hat(1);
    ys_hat(i) = state_hat(2);
    
    states_hat(1:5, i) = state_hat;
    
    es(i) = sqrt((state(X) - state_hat(X))^2 + (state(Y) - state_hat(Y))^2); 
end

FIELD_W = (26 + 11.25/12)*0.3048;
FIELD_H = (52 + 5.25/12)*0.3048;

figure(1);
plot(xs, ys);
hold on;
plot(xs_hat, ys_hat);
axis([-FIELD_W/2, FIELD_W/2, -FIELD_H/2, FIELD_H/2]);
daspect([1, 1, 1]);
legend(["actual", "estimated"]);
hold off;

figure(2);
plot(0:T:TMAX, states(vL:vR, 1:(1 + TMAX/T)));
title("Velocity over time");

figure(3);
plot(0:T:TMAX, states(THETA, 1:(1 + TMAX/T)));
title("Heading over time");

figure(5);
plot(0:T:TMAX, us);

figure(6);
plot(0:T:TMAX, states(4:5, 1:(TMAX/T + 1)));

figure(7);
plot(0:T:TMAX, es*12/0.3048);