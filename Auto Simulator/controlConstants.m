% drive motor constants
b = 4.093255526065792E-5;
R = 0.04666464678901319;
Kt = 0.01823032208047652;
Kw = 0.017857232059815025;
I = 6E-5;

ROBOT_RADIUS = 0.3048; % half of track width in meters
WHEEL_RADIUS = 0.0762; % in m

R = 1;
THETA_R = 2;
V = 3;
THETA = 4;
OMEGA = 5;

STATE_SIZE = 5;



F_LT = 1;
F_LB = 2;
F_RT = 3;
F_RB = 4;

INPUT_SIZE = 4;



M = 50; % in kg, 110 lbs approx
I_ROBOT = 1/2 * M * ROBOT_RADIUS^2; % assume spherical cow