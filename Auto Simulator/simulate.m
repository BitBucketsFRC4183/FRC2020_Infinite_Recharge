physicsConstants;

state = [
    -1; % x
    0.0; % y
    pi/4; % heading, theta
    0.00;
    0.00;
];

P = [
    0.1, 0, 0, 0, 0;
    0, 0.1, 0, 0, 0;
    0, 0, 5*pi/180, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0.005;
];

Q = [
    0.1, 0;
    0, 0.1;
];

R = [
    0.0508, 0, 0, 0, 0;
    0, 2*pi/180, 0, 0, 0;
    0, 0, 0.0001, 0, 0; % pretty certain about velocities
    0, 0, 0, 0.0001, 0;
    0, 0, 0, 0, 0.005;
];

state_hat = state;% + mvnrnd(zeros(STATE_SIZE, 1), 0.1*P, 1)';



u = [0; 0;];
T=0.01;

i = 0;

TMAX = 10;

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
    
    
    
    Ac = getABad(state);%getSysMatPhysics(state);
    Bc = getInpMatPhysics(state);
    Fc = zeros(STATE_SIZE, 1);%getFPhysics(state);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    sysdF = c2d(syscF, T);
    
    A = sysd.A;
    B = sysd.B;
    F = sysdF.B;

    G = B;
    
    lerr = state(vL) - vLfinal;
    rerr = state(vR) - vRfinal;
    
    %u(1) = kF*vLfinal + kP*lerr;
    %u(2) = kF*vRfinal + kP*rerr;
    
    %u(1) = 0;
    %u(2) = 0;

    %u(1) + 0.05*(2*rand-1);
    %u(2) + 0.05*(2*rand-1);
    
    us(1:2, i) = u;
    
    % move state forward in time
    %state = badUpdate(state, T);
    %state_ap = state;
    state = A*state + B*u + F;% + B*mvnrnd(zeros(2, 1), Q, 1)';
    %state(THETA) = mod(state(THETA), 2*pi);
    
    states(1:5, i) = state;
    
    
    
    %state_hat = state_ap;

    Ac = getABad(state_hat);%getSysMatPhysics(state_hat);
    Bc = getInpMatPhysics(state_hat);
    Fc = zeros(STATE_SIZE, 1);%getFPhysics(state_hat);

    sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
    sysd = c2d(sysc, T);
    
    syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
    sysdF = c2d(syscF, T);
    
    A = sysd.A;
    B = sysd.B;
    
    % predict
    state_hat = A*state_hat + B*u + sysdF.B;
    P = A*P*A' + G*Q*G';
    
    % measure
    y = getOutputPhysics(state) + mvnrnd(zeros(OUTPUT_SIZE, 1), R, 1)';
    
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
title("Velocity and estimated velocity over time");
hold on;
plot(0:T:TMAX, states_hat(vL:vR, 1:(1 + TMAX/T)));
hold off;

figure(3);
plot(0:T:TMAX, states(THETA, 1:(1 + TMAX/T))*180/pi);
hold on;
plot(0:T:TMAX, states_hat(THETA, 1:(1 + TMAX/T))*180/pi);
hold off;
title("Heading and estimated heading (deg) over time");

figure(4);
plot(0:T:TMAX, us);
title("Control effort over time");

figure(5);
plot(0:T:TMAX, es*100);
title("cm error over time");