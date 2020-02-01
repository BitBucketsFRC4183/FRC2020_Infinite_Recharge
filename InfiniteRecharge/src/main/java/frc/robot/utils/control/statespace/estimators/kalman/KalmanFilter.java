package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.estimators.LuenbergerObserver;
import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;

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
    /*
     * https://en.wikipedia.org/wiki/Kalman_filter#Predict
     * https://ocw.mit.edu/courses/mechanical-engineering/2-160-identification-estimation-and-learning-spring-2006/lecture-notes/lecture_6.pdf
     */
    public SimpleMatrix predict(double dt) {
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
        ABFTriple abf = MODEL.getLastSystem();

        // PA'(APA'+R)^-1
        SimpleMatrix K = P_apriori.mult(
            abf.getA().transpose()).mult(
                abf.getA().mult(P_apriori.mult(abf.getA().transpose()))
                .plus(getR(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime())
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