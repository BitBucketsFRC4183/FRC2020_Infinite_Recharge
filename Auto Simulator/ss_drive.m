physicsConstants;

Ac = zeros(4);
Ac(1, 3) = 1;
Ac(2, 4) = 1;
Ac(3:4, 3:4) = constants.A;

Bc = zeros(4, 2);
Bc(3:4, 1:2) = constants.B;