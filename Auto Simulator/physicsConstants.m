% drive motor constants
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

Am = (1/m + rb * rb / J);
Bm = (1/m - rb * rb / J);



% robot state
X = 1;
Y = 2;
THETA = 3;
vL = 4;
vR = 5;

% robot input
VL = 1;
VR = 2;

% system outputs
D = 1;
TX = 2;
VL_O = 3;
VR_O = 4;


STATE_SIZE = 5;
INPUT_SIZE = 2;
OUTPUT_SIZE = 4;


xT = 0.3048*5.58;
yT = 15.9830/2;

thetaLL = pi/6;
rLL = 0.2286;