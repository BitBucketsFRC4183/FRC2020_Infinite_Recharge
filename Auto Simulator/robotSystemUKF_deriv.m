function xDot = robotSystemUKF_deriv(x, u, c)
    xDot = zeros(c.STATE_SIZE, 1);
    
    xDot(c.X) = (x(c.vL) + x(c.vR))/2*cos(x(c.THETA));
    xDot(c.Y) = (x(c.vL) + x(c.vR))/2*sin(x(c.THETA));
    xDot(c.THETA) = 1/(2*c.rb)*(x(c.vR) - x(c.vL));
    xDot(c.vL:c.vR) = c.A*x(c.vL:c.vR) + c.B*u;
    % yawffset doesn't change
end

