% drive motor constants
b = 4.093255526065792E-5;
R = 0.04666464678901319;
Kt = 0.01823032208047652;
Kw = 0.017857232059815025;

constants.rb = 0.6070775928431744/2;
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
constants.X = 1;
constants.Y = 2;
constants.THETA = 3;
constants.vL = 4;
constants.vR = 5;

% robot input
constants.VL = 1;
constants.VR = 2;

% system outputs
constants.D = 1;
constants.TX = 2;
constants.VL_O = 3;
constants.VR_O = 4;
constants.LL_THETA = 5;
constants.OMEGA = 6;


constants.STATE_SIZE = 5;
constants.INPUT_SIZE = 2;
constants.OUTPUT_SIZE = 6;


constants.xT = 0.3048*5.58;
constants.yT = 15.9830/2;

constants.thetaLL = 0;%pi/6;
constants.rLL = 0;%0.2286;

constants.A = [
  -7.087195478354283   0.413285738104402
   0.339280393075371  -6.832080740045777
];

constants.B = [
   2.702895517197959  -0.241632861263366
  -0.126961060623545   2.551095849721741
];

Ay_c = zeros(3);
Ay_c(1:2, 1:2) = A;
Ay_c(3, 1) = 1/(2*rb);
Ay_c(3, 2) = -1/(2*rb);

By_c = zeros(3, 2);
By_c(1:2, 1:2) = B;

dt = 2/100; % 2ms default

sysy_c = ss(Ay_c, By_c, eye(3), zeros(3, 2));
sysy_d = c2d(sysy_c, dt);

constants.VQ = [
    0.006307812352729869, 0.006149384611897019;
    0.006149384611897019, 0.006451202836649863;
];

constants.IN_TO_M = 254 / 10000;

constants.yc = 0.6956 * IN_TO_M;
constants.xc = 6.7643 * IN_TO_M;
constants.rLL = 7.6252 * IN_TO_M;