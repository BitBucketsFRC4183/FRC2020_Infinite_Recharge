function y = robotSystemUKF_output(x, u, c)
    y = zeros(c.OUTPUT_SIZE, 1);
    
    xLL = x(c.X) + c.xc*cos(x(c.THETA)) - c.yc*sin(x(c.THETA)) + c.rLL*cos(x(c.THETA));
    yLL = y(c.X) + c.xc*sin(x(c.THETA)) + c.yc*cos(x(c.THETA)) + c.rLL*sin(x(c.THETA));
    
    d = sqrt((c.xT - xLL)^2 + (c.yT - yLL)^2);
    y(c.TY) = atan((c.targetHeight - c.cameraHeight) / d) - c.cameraAngle;
    % tan TX = y/x
    y(c.TX) = atan2(c.yT - x(c.Y), c.xT - x(c.X)) - x(c.THETA);
    y(c.VL_O) = x(c.vL);
    y(c.VR_O) = x(c.vR);
    y(c.LL_THETA) = x(c.THETA) + x(c.OFFSET);
    y(c.OMEGA) = (x(c.vR) - x(c.vL))/(2*c.rb);
end

