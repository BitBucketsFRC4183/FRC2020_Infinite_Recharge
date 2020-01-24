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
THETA = 3;

STATE_SIZE = 3;



OMEGA_LT = 1;
OMEGA_LB = 2;
OMEGA_RT = 3;
OMEGA_RB = 4;

INPUT_SIZE = 4;



M = 50; % in kg, 110 lbs approx
I_ROBOT = 1/2 * M * ROBOT_RADIUS^2; % assume spherical cow