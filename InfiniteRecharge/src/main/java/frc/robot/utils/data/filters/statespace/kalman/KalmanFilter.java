package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;
import frc.robot.utils.control.statespace.observer.LinearizedOutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.Noise;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/*
 * (Note: This class can be used as an Extended Kalman Filter too!)
 * 
 * Kalman Filters are an effective way to fuse data from multiple sensors to remove errors
 * Every sensor has some error, but by using the data in a very specific way, you can
 * filter out the most amount of error.
 * 
 * The Kalman Fitler is optimal for linear systems. Modifications such as the Extended
 * Kalman Filters (which linearizes about estimates) can be used for nonlinear systems, but
 * are guaranteed to be suboptimal but still retain a good amount of performance and are
 * (when compared to other methods) easy to implement.
 * 
 * Some very useful links to learn more:
 *   https://en.wikipedia.org/wiki/Kalman_filter#Predict
 *   https://en.wikipedia.org/wiki/Extended_Kalman_filter#Discrete-time_predict_and_update_equations
 *   https://ocw.mit.edu/courses/mechanical-engineering/2-160-identification-estimation-and-learning-spring-2006/lecture-notes/lecture_6.pdf
 *   https://www.mathworks.com/help/control/ug/kalman-filtering.html
 * Each of these links sucks and leaves out important things, but by reading all of them
 * you can get the entire truth. It's kind of like a Kalman Filter :)))
 */
public class KalmanFilter extends GenericKalmanFilter<LinearizedModel, LinearizedOutputObserver> {
    // just for convenience of not recalculating
    private SimpleMatrix C;
    // an identity matrix that will come in handy later
    private final SimpleMatrix I;

    /**
     * Create a Kalman filter for a linearized system
     * 
     * @param sys a state space system with linearized dynamics
     * @param P0 initial covariance (uncertainty) in state estimate
     */
    public KalmanFilter(StateSpaceSystem<LinearizedModel, LinearizedOutputObserver> sys, SimpleMatrix P0) {
        super(sys);

        P = P0;

        I = SimpleMatrix.identity(sys.getModel().getNumStates());
    }



    @Override
    protected SimpleMatrix predictState() {
        return SYS.propogate(SYS.getModel().getState());
    }

    @Override
    protected SimpleMatrix predictP() {
        LinearizedModel model = SYS.getModel();

        Noise processNoise = model.getNoiseSource().getNoise(
            model.getState(),
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        SimpleMatrix A = model.getLastSystem().getA();

        // x (covariance P) has gone through the transformation A
        // so new covariance (excluding noise) is APA'
        // but there is addative white noise
        // --> APA' + GQG'
        return A.mult(P).mult(A.transpose()).plus(processNoise.getCovariance());
    }

    @Override
    public void predict() {
        super.predict();

        // for convenience of not recalculating Jacobian

        LinearizedModel model = SYS.getModel();
        LinearizedOutputObserver outputObserver = SYS.getObserver();

        // set it in predict so it can be used in multiple methods in update
        C = outputObserver.getJacobian(
            x_apriori,
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );
    }



    @Override
    protected SimpleMatrix getS() {
        // x_apriori (covariance P_apriori) goes through transformation C
        // so the covariance (excluding noise) is C P_apriori C'
        // but there is addative white noise
        // C P_apriori C' + R
        LinearizedModel model = SYS.getModel();
        LinearizedOutputObserver outputObserver = SYS.getObserver();

        

        // must consider addative noise to sensor measurements
        Noise outputNoise = outputObserver.getNoiseSource().getNoise(
            x_apriori,
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        return C.mult(P_apriori).mult(C.transpose()).plus(outputNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getCxy() {
        // covariance of X is P_apriori and covariance of Y is C
        return P_apriori.mult(C.transpose());
    }

    @Override
    protected SimpleMatrix updateP() {
        return (I.minus(K.mult(C))).mult(P_apriori);
    }

    @Override
    protected SimpleMatrix getExpectedOutput() {
        // assuming linearity!
        return SYS.getOutput(x_apriori);
    }
}