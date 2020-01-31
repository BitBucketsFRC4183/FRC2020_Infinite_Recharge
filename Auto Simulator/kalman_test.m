b = 4.093255526065792E-5;
R = 0.04666464678901319;
Kt = 0.01823032208047652;
Kw = 0.017857232059815025;

rb = 0.3145536;
m = 39.3051396;
J = 2.1621037;
r = 0.0762;

G = 10.88888888888888888;
e = 1;%0.9;

C1 = -(Kt*Kw*G*G)/(R*r*r);
C2 = G*Kt/(R*r);

theta0 = 0.01;
v0 = 0.01;

Ac = [
    0, 0, -v0*sin(theta0), cos(theta0)/2, cos(theta0)/2;
    0, 0, v0*cos(theta0), sin(theta0)/2, sin(theta0)/2;
    0, 0, 0, -1/(2*rb), 1/(2*rb);
    0, 0, 0, (1/m + rb * rb / J)*C1, (1/m - rb * rb / J)*C1;
    0, 0, 0, (1/m - rb * rb / J)*C1, (1/m + rb * rb / J)*C1;
];

Bc = [
    0, 0;
    0, 0;
    0, 0;
    (1/m + rb * rb / J)*C2, (1/m - rb * rb / J)*C2;
    (1/m - rb * rb / J)*C2, (1/m + rb * rb / J)*C2;
];

sysc = ss(Ac, Bc, eye(5), zeros(5, 2));
sysd = c2d(sysc, 0.02);