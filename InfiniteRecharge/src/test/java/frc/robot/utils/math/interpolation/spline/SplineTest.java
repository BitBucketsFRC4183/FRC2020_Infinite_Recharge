package frc.robot.utils.math.interpolation.spline;

import org.junit.Test;

import static org.junit.Assert.*;

public class SplineTest {
    @Test
    public void testSpline() {
        double[] x = new double[] {1, 2, 3, 4, 5};
        double[] y = new double[] {3, 1, 4, 1, 5};

        Spline spline = new Spline(x, y);

        assertTrue(spline.get(0.5) == 3);
        assertTrue(spline.get(1) == 3);
        assertTrue(spline.get(2) == 1);
        assertTrue(spline.get(3) == 4);
        assertTrue(spline.get(4) == 1);
        assertTrue(spline.get(5) == 5);
        assertTrue(spline.get(5.5) == 5);
    }
}