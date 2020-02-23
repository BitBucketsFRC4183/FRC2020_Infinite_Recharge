package frc.robot.utils.math.interpolation.spline;

import org.junit.Test;

import frc.robot.utils.math.Polynomial;

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

        Polynomial[] polys = spline.getPolynomials();

        double tol = 0.0000001;
        
        // ensure differentiability at 3 inner knots
        assertEquals(polys[0].deriv().eval(1), polys[1].deriv().eval(0), tol);
        assertEquals(polys[1].deriv().eval(1), polys[2].deriv().eval(0), tol);
        assertEquals(polys[2].deriv().eval(1), polys[3].deriv().eval(0), tol);
    }
}