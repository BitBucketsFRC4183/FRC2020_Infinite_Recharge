package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.estimators.LuenbergerObserver;
import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;


/*
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
public abstract class KalmanFilter {
    private final LinearizedModel MODEL;

    private SimpleMatrix P;
    private SimpleMatrix P_apriori;

    private SimpleMatrix x_apriori;



    public KalmanFilter(SimpleMatrix P0, LinearizedModel model) {
        MODEL = model;
        this.P = P0;
        this.P_apriori = P0;
    }



    protected abstract SimpleMatrix getC(SimpleMatrix state, SimpleMatrix input, double t);
    protected abstract SimpleMatrix getD(SimpleMatrix state, SimpleMatrix input, double t);

    protected abstract SimpleMatrix getQ(SimpleMatrix state, SimpleMatrix input, double t);
    protected abstract SimpleMatrix getR(SimpleMatrix state, SimpleMatrix input, double t);
    protected abstract SimpleMatrix getG(SimpleMatrix state, SimpleMatrix input, double t);



    /**
     * 
     * @param dt
     * @return
     */
    public SimpleMatrix predict() {
        // model already has a state, so predict based on that
        x_apriori = MODEL.update();

        ABFTriple abf = MODEL.getLastSystem();
        P_apriori = 
            // APA' + Q
            abf.getA().mult(P).mult(
                abf.getA().transpose()
                .plus(getR(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime()))
            );

        return x_apriori;
    }

    public SimpleMatrix update(SimpleMatrix y) {
        SimpleMatrix C = getC(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime());
        SimpleMatrix R = getR(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime());

        // PC'(CPC'+R)^-1
        SimpleMatrix K = P_apriori.mult(
            C.transpose().mult(
                C.mult(P_apriori.mult(C.transpose()))
                .plus(R)
                .invert()
            )
        );

        SimpleMatrix x_posteriori = x_apriori.plus(
            K.mult(
                y.minus(
                    getC(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime())
                )
            )
        );

        MODEL.setState(x_posteriori);

        return x_posteriori;
    }



    public SimpleMatrix getOutput(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return
            // y = Cx + Du
            getC(stateVector, inputVector, t).mult(stateVector)
            .plus(getD(stateVector, inputVector, t).mult(inputVector));
    }
    
}