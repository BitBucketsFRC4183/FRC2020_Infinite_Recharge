package frc.robot.utils.control.statespace.estimators.kalman;

import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import frc.robot.utils.control.statespace.models.motor.MotorPosition;
import frc.robot.utils.control.statespace.models.motors.MotorType;



public class KalmanFilterTest {
    private static final double TOLERANCE = 0.000001;

    @Test
    public void motor() {
        double t0 = System.nanoTime();

        double I = 0.001;
        MotorType type = MotorType.Falcon500;
        double dt = 0.02;



        MotorPosition model = new MotorPosition(dt, type, I);

        model.setState(new SimpleMatrix(new double[][] {
            new double[] {0},
            new double[] {0}
        }));

        SimpleMatrix P0 = new SimpleMatrix(new double[][] {
            new double[] {0.01, 0},
            new double[] {0, 0.02},
        });



        SimpleMatrix R = new SimpleMatrix(new double[][] {
            new double[] {0.03, 0},
            new double[] {0, 0.04},
        });

        SimpleMatrix Q = new SimpleMatrix(new double[][] {
            new double[] {0.05, 0},
            new double[] {0, 0.06},
        });

        SimpleMatrix G = new SimpleMatrix(new double[][] {
            new double[] {1, 2},
            new double[] {3, 4}
        });

        // idk what kind of encoder gives us this but don't judge
        SimpleMatrix C = new SimpleMatrix(new double[][] {
            new double[] {2, 3},
            new double[] {-0.1, 5}
        });

        SimpleMatrix D = new SimpleMatrix(2, 2);

        KalmanFilter filter = new KalmanFilter(P0, model) {
            @Override
            protected SimpleMatrix getR(SimpleMatrix state, SimpleMatrix input, double t) {
                return R;
            }
        
            @Override
            protected SimpleMatrix getQ(SimpleMatrix state, SimpleMatrix input, double t) {
                return Q;
            }
        
            @Override
            protected SimpleMatrix getG(SimpleMatrix state, SimpleMatrix input, double t) {
                return G;
            }
        
            @Override
            protected SimpleMatrix getD(SimpleMatrix state, SimpleMatrix input, double t) {
                return D;
            }
        
            @Override
            protected SimpleMatrix getC(SimpleMatrix state, SimpleMatrix input, double t) {
                return C;
            }
        };

        model.setState(new SimpleMatrix(2, 1, true, new double[] {Math.PI, Math.PI / 100}));



        model.apply(new SimpleMatrix(1, 1, true, new double[] {8}));
        filter.predict();

        SimpleMatrix y = new SimpleMatrix(2, 1, true, new double[] {181, 295});
        filter.update(y);

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            2.650741871503023, 58.865022680661937
        }), TOLERANCE));



        model.apply(new SimpleMatrix(1, 1, true, new double[] {6}));
        filter.predict();

        y = new SimpleMatrix(2, 1, true, new double[] {295, 470});
        filter.update(y);

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            5.338061309893673, 94.364827040089935
        }), TOLERANCE));

        double t1 = System.nanoTime();

        System.out.println((t1 - t0) / 1000000 + "ms to run");
    }

    @Test
    public void motor_time() {
        for (int i = 0; i < 100; i++) {
            motor();
        }
    }
}