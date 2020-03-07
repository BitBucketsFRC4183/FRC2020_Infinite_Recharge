package frc.robot.utils.data.filters.statespace.kalman;

import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;
import frc.robot.utils.control.statespace.models.motor.MotorPosition;
import frc.robot.utils.control.statespace.models.motors.MotorType;
import frc.robot.utils.control.statespace.observer.LinearizedOutputObserver;
import frc.robot.utils.control.statespace.observer.SimpleOutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.ConstantNoiseSource;



public class KalmanFilterTest {
    private static final double TOLERANCE = 0.000001;

    @Test
    public void motor() {
        double t0 = System.nanoTime();

        double I = 0.001;
        MotorType type = MotorType.Falcon500;
        double dt = 0.02;



        MotorPosition model = new MotorPosition(dt, type, I);
        model.addNoiseSource(new ConstantNoiseSource(
            2,
            2,
            new SimpleMatrix(new double[][] {
                new double[] {1, 2},
                new double[] {3, 4}
            }),
            new SimpleMatrix(new double[][] {
                new double[] {0.05, 0},
                new double[] {0, 0.06},
            })
        ));



        SimpleOutputObserver observer = new SimpleOutputObserver(
            2,
            new SimpleMatrix(new double[][] {
                new double[] {2, 3},
                new double[] {-0.1, 5}
            }),
            new SimpleMatrix(2, 1)
        );
        observer.addNoiseSource(new ConstantNoiseSource(
            2,
            new SimpleMatrix(new double[][] {
                new double[] {0.03, 0},
                new double[] {0, 0.04},
            })
        ));



        StateSpaceSystem<LinearizedModel, LinearizedOutputObserver> sys = new StateSpaceSystem<LinearizedModel, LinearizedOutputObserver>(model, observer);



        KalmanFilter filter = new KalmanFilter(
            sys,
            new SimpleMatrix(new double[][] {
                new double[] {0.01, 0},
                new double[] {0, 0.02},
            })
        );

        sys.setFilter(filter);
        sys.getModel().setState(new SimpleMatrix(2, 1, true, new double[] {Math.PI, Math.PI / 100}));



        sys.apply(new SimpleMatrix(1, 1, true, new double[] {8}));
        sys.observe(new SimpleMatrix(2, 1, true, new double[] {181, 295}));

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            2.650741871503023, 58.865022680661937
        }), TOLERANCE));



        sys.apply(new SimpleMatrix(1, 1, true, new double[] {6}));
        sys.observe(new SimpleMatrix(2, 1, true, new double[] {295, 470}));

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            5.338061309893673, 94.364827040089935
        }), TOLERANCE));

        double t1 = System.nanoTime();

        System.out.println((t1 - t0) / 1000000 + "ms to run");
    }
}