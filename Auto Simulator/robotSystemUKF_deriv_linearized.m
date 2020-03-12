function A = robotSystemUKF_deriv_linearized(x, c)
    A = zeros(c.STATE_SIZE, c.STATE_SIZE);
    
    A(c.X, c.vL) = 1/2*cos(x(c.THETA));
    A(c.X, c.vR) = 1/2*cos(x(c.THETA));
    A(c.X, c.THETA) = -(x(c.vL) + x(c.vR))/2*sin(x(c.THETA));
    
    A(c.Y, c.vL) = 1/2*sin(x(c.THETA));
    A(c.Y, c.vR) = 1/2*sin(x(c.THETA));
    A(c.Y, c.THETA) = (x(c.vL) + x(c.vR))/2*cos(x(c.THETA));
    
    A(c.THETA, c.vR) = 1/(2*c.rb);
    A(c.THETA, c.vL) = -1/(2*c.rb);
    
    A(c.vL:c.vR, c.vL:c.vR) = c.A;
end

