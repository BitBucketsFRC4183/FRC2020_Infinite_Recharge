G = [
    1, 2, 3, 4, 5;
    6, 7, 8, 9, 10;
    11, 12, 13, 14, 15;
    16, 17, 18, 19, 20;
    21, 22, 23, 24, 25;
];

Q = [
    1.1, 0, 0, 0, 0;
    0, 1.2, 0, 0, 0;
    0, 0, 1.3, 0, 0;
    0, 0, 0, 1.4, 0;
    0, 0, 0, 0, 1.5;
];

R = [
    0.1, 0, 0, 0, 0;
    0, 0.2, 0, 0, 0;
    0, 0, 0.3, 0, 0;
    0, 0, 0, 0.4, 0;
    0, 0, 0, 0, 0.5;
];

P = [
    1, 0, 0, 0, 0;
    0, 2, 0, 0, 0;
    0, 0, 6, 0, 0;
    0, 0, 0, 4, 0;
    0, 0, 0, 0, 5;
];



state = [1; 2; pi/6; 3; 4;];

Ac = getSysMatPhysics(state);
Bc = getInpMatPhysics(state);
Fc = getFPhysics(state);

sysc = ss(Ac, Bc, eye(STATE_SIZE), zeros(STATE_SIZE, INPUT_SIZE));
sysd = c2d(sysc, T);

syscF = ss(Ac, Fc, eye(STATE_SIZE), zeros(STATE_SIZE, 1));
sysdF = c2d(syscF, T);

Ad = sysd.A;
Bd = sysd.B;
Fd = sysdF.B;

C = eye(5);

u = [0.5; -0.2;];



% predict
state = Ad*state + Bd*u + Fd;
P = Ad*P*Ad' + G*Q*G';

% measurements
y = [1.069; 2.047; 0.53; 2.5; pi;];

% update
K = P*C'/(C*P*C'+R);
state = state + K * (y - C * state);
P = (eye(5) - K*C)*P;
