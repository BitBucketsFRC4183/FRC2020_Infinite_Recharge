dt = 0.02;
ts = 0:dt:15;
us = [2 + 0.1*ts; pi/2 - pi/45*ts];

t = 0;
u = us(:, 1);

h = @(x) [sqrt(x(1).^2 + x(2).^2); x(3); x(4);];

Q = diag([0.01, 0.01, 0.05, pi/180]);
R = diag([0.05, 0.05, pi/45]);
P = Q;

deriv = @(t, x, U) [x(3)*cos(x(4)); x(3)*sin(x(4)); U(1); U(2)];
f = @(x) simple_robot_ukf_update(deriv, [t, t+dt], x, u);

x_hat = [1; 2; 0.5; pi / 3];
x0 = x_hat + mvnrnd([0; 0; 0; 0;], P)';

AbsTol = [0.01; 0.01; 0.02; pi / 90];
RelTol = AbsTol;

[~, t_width] = size(ts);

Xs = zeros(t_width);
Ys = zeros(t_width);
Xs_hat = zeros(t_width);
Ys_hat = zeros(t_width);

for i=1:t_width
    u = us(:, i);
    t = ts(i);
    
    x0 = f(x0) + mvnrnd([0; 0; 0; 0;], Q)';
    y = h(x0) + mvnrnd([0; 0; 0;], R)';
    
    [x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);
    
    Xs(i) = x0(1);
    Ys(i) = x0(2);
    Xs_hat(i) = x_hat(1);
    Ys_hat(i) = x_hat(2);
end

figure(1)
plot(Xs, Ys);
hold on;
plot(Xs_hat, Ys_hat);
hold off;