close all;

physicsConstants;



dt = 2/100;

ts = 0:dt:1;
us = 6 + 6*[sin(ts); cos(ts)];
[~, t_width] = size(ts);

t = 0;
u = us(:, 1);

f = @(x) robotSystemUKF_update(@robotSystemUKF_deriv, [t, t+dt], x, u);
h = @(x) robotSystemUKF_output(x, u);

Q = diag([0.01, 0.01, pi/180, 0.02, 0.02])/50;
R = diag([254/10000, pi/90, 0.005, 0.005, 0.5*pi/180, pi/36]);
P = diag([254/10000*1, 254/10000*1, 2*pi/180, 0.0001, 0.0001]);

x_hat = [1; 1; pi/2; 0; 0;];
x0 = x_hat;% + mvnrnd(zeros(STATE_SIZE, 1), P)';

AbsTol = [0.01; 0.01; pi / 90; 0.05; 0.05];
RelTol = AbsTol;

Xs = zeros(t_width, 1);
Ys = zeros(t_width, 1);
Xs_hat = zeros(t_width, 1);
Ys_hat = zeros(t_width, 1);
Thetas = zeros(t_width, 1);
Thetas_hat = zeros(t_width, 1);
Thetas_m = zeros(t_width, 1);
Eigs = zeros(t_width, STATE_SIZE);
Outputs = zeros(t_width, OUTPUT_SIZE);
Outputs_hat = zeros(t_width, OUTPUT_SIZE);
Outputs_exact = zeros(t_width, OUTPUT_SIZE);

for i=1:t_width
    u = us(:, i);
    t = ts(i);
    
    x0 = f(x0) + mvnrnd(zeros(STATE_SIZE, 1), Q)';
    y0 = h(x0);
    y = y0 + mvnrnd(zeros(OUTPUT_SIZE, 1), R)';
    %y(TX) = y(TX) - 5*pi/180;
    
    [x_hat, P] = ukf(f, x_hat, P, h, y, Q, R);
    
    Xs(i) = x0(X);
    Ys(i) = x0(Y);
    Xs_hat(i) = x_hat(X);
    Ys_hat(i) = x_hat(Y);
    Thetas(i) = x0(THETA);
    Thetas_hat(i) = x_hat(THETA);
    %Thetas_m(i) = y(LL_THETA);
    Eigs(i, :) = eig(P);
    Outputs(i, :) = y;
    Outputs_hat(i, :) = h(x_hat);
    Outputs_exact(i, :) = y0;
end



figure(1)
hold on;
plot(Xs, Ys);
plot(Xs_hat, Ys_hat);
hold off;
legend("True position", "Estimated position");

figure(2)
hold on;
plot(ts, Thetas);
plot(ts, Thetas_hat);
plot(ts, Thetas_m);
hold off;
legend("True yaw", "Estimated yaw", "Measured yaw");

figure(3);
hold on;
plot(ts, sqrt((Xs-Xs_hat).^2 + (Ys-Ys_hat).^2));
yline(0.1); % 10cm
hold off;
title("Localization error in meters");

figure(4);
plot(ts, Eigs);
title("Uncertainty eigenvalues");

%figure(5);
%hold on;
%plot(ts, Outputs);
%plot(ts, Outputs_hat);
%plot(ts, Outputs_exact);
%hold off;
%legend1 = [];
%legend2 = [];
%legend3 = [];
%for i=1:OUTPUT_SIZE
%    legend1 = [legend1, "Output"];
%    legend2 = [legend2, "Output estimate"];
%    legend3 = [legend3, "Actual output"];
%end
%legend([legend1, legend2, legend3]);