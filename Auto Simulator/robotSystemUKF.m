close all;

physicsConstants;



T = 5;
ts = 0:dt:T;
[~, t_width] = size(ts);
us = 6 + 6*[sin(ts); cos(ts)];
%us = rand(2, t_width)*6;

t = 0;
u = us(:, 1);

deriv = @(x, u) robotSystemUKF_deriv(x, u, constants);
f = @(x) robotSystemUKF_update(deriv, [t, t+dt], x, u);
h = @(x) robotSystemUKF_output(x, u, constants);

%Q = diag([0.0005, 0.0005, 0.005*pi/180, 0.02, 0.02, 0.00000001])/50;
%Q(constants.VL:constants.VR, constants.VL:constants.VR) = constants.VQ;
R = diag([254/10000, pi/90, 0.005, 0.005, 0.5*pi/180, pi/36]);
P = diag([254/10000*1, 254/10000*1, 2*pi/180, 0.0001, 0.0001, 2*pi/180]);

x_hat = [1; 1; pi/2; 0; 0; 0;];
%x0 = [1.05, 0.95, pi/2 + pi/12, 0, 0, 5*pi/180]';
e = 100;
while (e >= 4*constants.IN_TO_M)
    x0 = x_hat + mvnrnd(zeros(constants.STATE_SIZE, 1), P)';
    
    e = sqrt((x0(constants.X) - x_hat(constants.X))^2 + (x0(constants.Y) - x_hat(constants.Y))^2);
end
disp(x0);

AbsTol = [0.01; 0.01; pi / 90; 0.05; 0.05];
RelTol = AbsTol;

Xs = zeros(t_width, 1);
Ys = zeros(t_width, 1);
Xs_hat = zeros(t_width, 1);
Ys_hat = zeros(t_width, 1);
Thetas = zeros(t_width, 1);
Thetas_hat = zeros(t_width, 1);
Thetas_m = zeros(t_width, 1);
Thetas_mf = zeros(t_width, 1);
Eigs = zeros(t_width, constants.STATE_SIZE);
Outputs = zeros(t_width, constants.OUTPUT_SIZE);
Outputs_hat = zeros(t_width, constants.OUTPUT_SIZE);
Outputs_exact = zeros(t_width, constants.OUTPUT_SIZE);

Qc = zeros(constants.STATE_SIZE, constants.STATE_SIZE);
Qc(constants.vL:constants.vR, constants.vL:constants.vR) = constants.VQc;

for i=1:t_width
    u = us(:, i);
    t = ts(i);
    
    % estimate process noise covariance
    % noise source comes from velocity noise
    Ac = robotSystemUKF_deriv_linearized(x_hat, constants);
    sysc = ss(Ac, zeros(constants.STATE_SIZE), zeros(1, constants.STATE_SIZE), zeros(1, constants.STATE_SIZE));
    Ad = c2d(sysc, dt).A;
    F = [-Ac, Qc; zeros(constants.STATE_SIZE), Ac'];
    G = expm(F*dt);
    Q = Ad*G(1:constants.STATE_SIZE, constants.STATE_SIZE+1:2*constants.STATE_SIZE);
    % so basically do an unscented transform
    % to estimate position and yaw noise
    %QQ = eye(STATE_SIZE)*0.00000001;
    %QQ(vL:vR, vL:vR) = VQ;
    %v_sigmas = sigmas(x0, QQ, c);
    %[~, ~, Q, ~] = ut(f, x_sigmas, Wm, Wc, STATE_SIZE, zeros(STATE_SIZE));

    x0 = f(x0) + mvnrnd(zeros(constants.STATE_SIZE, 1), Q)';
    y0 = h(x0);
    y = y0 + mvnrnd(zeros(constants.OUTPUT_SIZE, 1), R)';
    %y(TX) = y(TX) - 5*pi/180;
    
    [x_hat, P, K] = ukf(f, x_hat, P, h, y, Q, R);
    
    Xs(i) = x0(constants.X);
    Ys(i) = x0(constants.Y);
    Xs_hat(i) = x_hat(constants.X);
    Ys_hat(i) = x_hat(constants.Y);
    Thetas(i) = x0(constants.THETA);
    Thetas_hat(i) = x_hat(constants.THETA);
    Thetas_m(i) = y(constants.LL_THETA);
    Thetas_mf(i) = y0(constants.LL_THETA);
    Eigs(i, :) = eig(P);
    Outputs(i, :) = y;
    Outputs_hat(i, :) = h(x_hat);
    Outputs_exact(i, :) = y0;
end

err = x0 - x_hat;
disp("Yawffset error: " + err(constants.OFFSET)*180/pi + " degrees");



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
plot(ts, Thetas_mf);
hold off;
legend("True yaw", "Estimated yaw", "Measured yaw", "Theoretical measured yaw");

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