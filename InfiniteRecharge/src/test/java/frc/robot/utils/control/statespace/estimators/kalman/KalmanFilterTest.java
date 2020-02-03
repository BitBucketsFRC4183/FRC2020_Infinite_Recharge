package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import frc.robot.utils.control.statespace.models.motor.MotorPosition;
import frc.robot.utils.control.statespace.models.motors.MotorType;



public class KalmanFilterTest {
    @Test
    public void motor() {
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

        SimpleMatrix C = new SimpleMatrix(new double[][] {
            new double[] {1, 0},
            new double[] {0, 1}
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

        model.apply(new SimpleMatrix(1, 1, true, new double[] {8}));
    }
}