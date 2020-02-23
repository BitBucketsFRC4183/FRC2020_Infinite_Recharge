f = @(x) [x(1).^2 + abs(x(2)); sin(x(2)) - sqrt(x(1))];

h = @(x) [tan(x(1) + x(2)); 1/x(2)];

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
x = x_hat; % for simplicity

x = f(x);
y = [-0.23; -11];%h(x) + mvnrnd([0; 0;], R, 1)';
[x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);

x_hat

x = f(x);
y = [pi/2; -0.54];
[x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);

x_hat