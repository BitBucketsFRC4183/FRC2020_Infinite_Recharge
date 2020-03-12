package frc.robot.utils.math;

import org.junit.Test;

import static org.junit.Assert.*;

import org.ejml.simple.SimpleMatrix;

public class MathUtilsTest {
    private static final double TOLERANCE = 0.0000001;

    @Test
    public void testUnitConverter() {
        // convert 6000 rpm to ticks per 100ms
        // from the math, 6000 RPMs should come out to 20480 ticks per 100ms
        // assuming 2048 ticks per revolution
        assertEquals(20480, MathUtils.unitConverter(6000, 600, 2048), 0);

        // test converting degrees to ticks assuming 4096 ticks per revolution
        // 90 degrees is 1/4 of a rotation, or 1024 ticks
        assertEquals(1024, MathUtils.unitConverter(90, 360, 4096), 0);
    }

    @Test
    public void testPositiveDefiniteCholesky() {
        SimpleMatrix Q = new SimpleMatrix(2, 2, true, new double[] {
            0.006307812352730, 0.006149384611897,
            0.006149384611897, 0.006451202836650
        });
        
        SimpleMatrix P = MathUtils.chol(Q, false);

        assertTrue(P.isIdentical(new SimpleMatrix(2, 2, true, new double[] {
            0.079421737280985, 0.077426971789111,
            0,                 0.021360404402027
        }), TOLERANCE));

        P = MathUtils.chol(Q, true);

        assertTrue(P.isIdentical(new SimpleMatrix(2, 2, true, new double[] {
            0.079421737280985,                   0,
            0.077426971789111,   0.021360404402027
        }), TOLERANCE));
    }

    // positive semi definite matrix Cholesky decomposition isn't supported yet
    // (coming Soon(TM))
}
