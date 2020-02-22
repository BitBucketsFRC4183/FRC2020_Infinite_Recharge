physicsConstants;



dt = 0.02;

ts = 0:dt:1;
us = 6 + 6*[sin(ts); cos(ts)];
[~, t_width] = size(ts);

t = 0;
u = us(:, 1);

f = @(x) robotSystemUKF_update(@robotSystemUKF_deriv, [t, t+dt], x, u);
h = @(x) robotSystemUKF_output(x, u);

Q = diag([0.01, 0.01, pi/180, 0.02, 0.02])/50;
R = diag([254/10000, pi/90, 0.005, 0.005, pi/45, pi/36]);
P = Q;

x_hat = [1; 1; pi/2; 0; 0;];
x0 = x_hat;% + mvnrnd(zeros(STATE_SIZE, 1), P)';

AbsTol = [0.01; 0.01; pi / 90; 0.05; 0.05];
RelTol = AbsTol;

Xs = zeros(t_width);
Ys = zeros(t_width);
Xs_hat = zeros(t_width);
Ys_hat = zeros(t_width);

for i=1:t_width
    u = us(:, i);
    t = ts(i);
    
    x0 = f(x0) + mvnrnd(zeros(STATE_SIZE, 1), Q)';
    y = h(x0) + mvnrnd(zeros(OUTPUT_SIZE, 1), R)';
    
    [x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);
    
    Xs(i) = x0(X);
    Ys(i) = x0(Y);
    Xs_hat(i) = x_hat(X);
    Ys_hat(i) = x_hat(Y);
end



figure(1)
plot(Xs, Ys);
hold on;
plot(Xs_hat, Ys_hat);
hold off;
legend(["True", "Estimated"]);

figure(2);
plot(ts, sqrt((Xs-Xs_hat).^2 + (Ys-Ys_hat).^2));