% drive motor constants
b = 4.093255526065792E-5;
R = 0.04666464678901319;
Kt = 0.01823032208047652;
Kw = 0.017857232059815025;
I = 6E-5;

ROBOT_RADIUS = 0.3048; % half of track width in meters
WHEEL_RADIUS = 0.0762; % in m

X = 1;
Y = 2;
THETA = 3;
OMEGA_LT = 4;
OMEGA_LB = 5;
OMEGA_RT = 6;
OMEGA_RB = 7;

STATE_SIZE = 7;



VOLTS_LT = 1;
VOLTS_LB = 2;
VOLTS_RT = 3;
VOLTS_RB = 4;

INPUT_SIZE = 4;



M = 50; % in kg, 110 lbs approx
I_ROBOT = 1/2 * M * ROBOT_RADIUS^2; % assume spherical cow





F_W = -1/WHEEL_RADIUS * (Kt * Kw / R + b);
F_V = 1/WHEEL_RADIUS * Kt/R;

W_W = -(Kt * Kw / R + b) / I;
W_V = Kt / (R * I);