f = @(x) [x(1).^2 + abs(x(2)); sin(x(2)) - sqrt(x(1))];

h = @(x) [tan(x(1)); 1/x(2)];

Q = [
    3, 2;
    2, 4
];

R = [
    1, 0;
    0, 4;
];

P = [
    0.05, 0;
    0, 0.1;
];

x_hat = [1; 2;];
x = x_hat + mvnrnd([0; 0;], P, 1)';

x = f(x) + mvnrnd([0; 0;], Q, 1)';
y = h(x) + mvnrnd([0; 0;], R, 1)';
[x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);