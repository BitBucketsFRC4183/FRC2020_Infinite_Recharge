package frc.robot.utils.control.statespace.models;

import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;



public class CStateSpaceModelTest {
    private static double TOLERANCE = 0.042643518606546;

    @Test
    public void simpleBot() {
        double t0 = System.nanoTime();

        // x' = v cos theta
        // y' = v sin theta
        // v' = a --> control
        // theta' = omega --> control
        CStateSpaceModel model = new CStateSpaceModel(
            4, // 4 states
            2, // 2 inputs
            10, // 20s
            0.001, // min 1ms step
            0.005, // max 5ms step
            new double[] {0.01, 0.01, 0.02, Math.PI / 90},
            new double[] {0.01, 0.01, 0.02, Math.PI / 90} // idek man, why not just use absoluteeee
        ) {
            @Override
            public void deriv(double[] x, double[] u, double t, int k, double[] xDot) {
                xDot[0] = x[2] * Math.cos(x[3]);
                xDot[1] = x[2] * Math.sin(x[3]);

                xDot[2] = u[0];
                xDot[3] = u[1];
            }
        };

        SimpleMatrix newState = model.update(
            new SimpleMatrix(4, 1, true, new double[] {1, 2, 0.5, Math.PI / 3}),
            new SimpleMatrix(2, 1, true, new double[] {2, Math.PI / 2}),
            0,
            0
        );

        assertTrue(
            newState.isIdentical(new SimpleMatrix(4, 1, true, new double[] {
                // there's probably an analytic solution, but this is good enough
                // MATLAB returned these with its ode45, which has a slightly
                // different implementation I believe
                // however, both should have |e(x)| < |AbsTol| where |e(x)| is
                // the magnitude of the error vector
                // thus, their difference must be within 2*|AbsTol|
                // (think of two points in a circle
                // of radius r - max distance between them is diameter = 2r)
                -11.388268047410813,
                7.280311066184881,
                20.499999999999996,
                16.755160819145566
            }), 2*TOLERANCE)
        );

        double t1 = System.nanoTime();

        System.out.println((t1 - t0) / 1000000 + "ms to run");
    }
}