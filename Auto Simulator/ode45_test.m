u = [2; pi/2;];

f = @(t, x) [x(3)*cos(x(4)); x(3)*sin(x(4)); u(1); u(2)];

x0 = [1; 2; 0.5; pi / 3];
ts = [0 10];
AbsTol = [0.01; 0.01; 0.02; pi / 90];
RelTol = AbsTol;

%ptions = odeset("AbsTol", norm(AbsTol), "RelTol", norm(RelTol));
[ts2, x] = ode45(f, ts, x0);%, options);

figure(1);
plot(ts2, x');

figure(2);
plot(x(:,1), x(:,2));

[steps, ~] = size(x);
x(steps, :)'