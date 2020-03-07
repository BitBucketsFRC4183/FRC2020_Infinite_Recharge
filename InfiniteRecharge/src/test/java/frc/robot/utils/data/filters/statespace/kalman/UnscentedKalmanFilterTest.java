package frc.robot.utils.data.filters.statespace.kalman;

import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.ConstantNoiseSource;

public class UnscentedKalmanFilterTest {
    private static final double TOLERANCE = 0.000001;



    @Test
    // creatively named
    public void testUKF() {
        double t0 = System.nanoTime();

        StateSpaceModel model = new StateSpaceModel(2, 0) {
            @Override
            public SimpleMatrix update(SimpleMatrix stateVector, SimpleMatrix inputVector, double t, int k) {
                return new SimpleMatrix(2, 1, true, new double[] {
                    Math.pow(stateVector.get(0), 2) + Math.abs(stateVector.get(1)),
                    Math.sin(stateVector.get(1)) - Math.sqrt(stateVector.get(0)),
                });
            }
        };
        model.addNoiseSource(new ConstantNoiseSource(
            2,
            new SimpleMatrix(2, 2, true, new double[] {
                3, 2,
                2, 4
            })
        ));
        model.setState(new SimpleMatrix(2, 1, true, new double[] {1, 2}));

        OutputObserver observer = new OutputObserver(2) {
            @Override
            public SimpleMatrix getOutput(SimpleMatrix state, SimpleMatrix input, double t, int k) {
                return new SimpleMatrix(2, 1, true, new double[] {
                    Math.tan(state.get(0) + state.get(1)),
                    1/state.get(1)
                });
            }
        };
        observer.addNoiseSource(new ConstantNoiseSource(
            2,
            new SimpleMatrix(2, 2, true, new double[] {
                1, 0,
                0, 4
            })
        ));

        StateSpaceSystem<StateSpaceModel, OutputObserver> sys = new StateSpaceSystem<StateSpaceModel, OutputObserver>(model, observer);

        UnscentedKalmanFilter filter = new UnscentedKalmanFilter(
            sys,
            new SimpleMatrix(2, 2, true, new double[] {
                0.05, 0,
                0, 0.1
            })
        );
        
        sys.setFilter(filter);



        sys.apply(new SimpleMatrix(0, 1));
        sys.observe(new SimpleMatrix(2, 1, true, new double[] {-0.23, -11}));

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            3.127348145045212,
            -0.135789727039768
        }), TOLERANCE));



        sys.apply(new SimpleMatrix(0, 1));
        sys.observe(new SimpleMatrix(2, 1, true, new double[] {Math.PI/2, -0.54}));

        assertTrue(model.getState().isIdentical(new SimpleMatrix(2, 1, true, new double[] {
            9.994466480058852,
            -1.897007287825946
        }), TOLERANCE));

        double t1 = System.nanoTime();

        System.out.println((t1 - t0) / 1000000 + "ms to run");
    }
}