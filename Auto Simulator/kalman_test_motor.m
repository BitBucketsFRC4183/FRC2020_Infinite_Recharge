physicsConstants;

I = 0.001;

Ac = [
    0, 1;
    0, -((Kt*Kw)/R + b)/I
];

Bc = [
    0;
    Kt/(I*R);
];

C = [
    2, 3;
    -0.1, 5;
];

T = 0.02;

sysc = ss(Ac, Bc, C, zeros(2, 1));
sysd = c2d(sysc, T);



G = [
    1, 2;
    3, 4;
];

Q = [
    0.05, 0;
    0, 0.06;
];

R = [
    0.03, 0;
    0, 0.04;
];

P = [
    0.01, 0;
    0, 0.02;
];



Ad = sysd.A;
Bd = sysd.B;



state = [pi; pi/100;];
u = 8;

% predict
state = Ad*state + Bd*u;
disp("a priori x");
state
P = Ad*P*Ad' + G*Q*G';
P

% measurements
y = [181; 295];

% update
S = (C*P*C'+R);
S
Cxy = P*C';
Cxy
K = Cxy/S;
K
C * state
state = state + K * (y - C * state); % posteriori estimate
P = (eye(2) - K*C)*P;
P
disp("posteriori state 1");
state



u = 6;
state = Ad*state + Bd*u;
P = Ad*P*Ad' + G*Q*G';
disp("a priori state 2");
state
y = [295; 470;];

K = P*C'/(C*P*C'+R);
state = state + K * (y - C * state); % posteriori estimate
disp("posteriori state 2");
state