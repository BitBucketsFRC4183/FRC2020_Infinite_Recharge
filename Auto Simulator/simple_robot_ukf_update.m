function x3 = simple_robot_ukf_update(f, ts, x1, u)
    f_u = @(T, X) f(T, X, u);
    [~, x2] = ode45(f_u, ts, x1);
    [steps, ~] = size(x2);
    x3 = x2(steps, :)';
end

