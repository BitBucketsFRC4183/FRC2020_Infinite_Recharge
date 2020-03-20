constants.rb = 0.6070775928431744/2;

% robot state
constants.X = 1;
constants.Y = 2;
constants.THETA = 3;
constants.vL = 4;
constants.vR = 5;
constants.OFFSET = 6; % alternatively, YAWFFSET

% robot input
constants.VL = 1;
constants.VR = 2;

% system outputs
constants.TY = 1;
constants.TX = 2;
constants.VL_O = 3;
constants.VR_O = 4;
constants.LL_THETA = 5;
constants.OMEGA = 6;


constants.STATE_SIZE = 6;
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

dt = 2/100; % 2ms default

constants.VQ = [
    0.006307812352729869, 0.006149384611897019;
    0.006149384611897019, 0.006451202836649863;
];

constants.VQc = [
    0.359432446406913, 0.349630210052408;
    0.349630210052408, 0.366367970306715;
];

constants.IN_TO_M = 254 / 10000;

constants.yc = 0.6956 * constants.IN_TO_M;
constants.xc = 6.7643 * constants.IN_TO_M;
constants.rLL = 7.6252 * constants.IN_TO_M;

constants.targetHeight = 89.75 * constants.IN_TO_M;
constants.cameraHeight = 22.6 * constants.IN_TO_M;
constants.cameraAngle = 30 * pi/180;