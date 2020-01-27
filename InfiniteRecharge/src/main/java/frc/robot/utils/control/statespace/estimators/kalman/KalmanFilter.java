package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.estimators.LuenbergerObserver;
import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;

public abstract class KalmanFilter {
    private final LinearizedModel MODEL;

    private SimpleMatrix P;
    private SimpleMatrix P_next;

    private SimpleMatrix xhat_next;



    public KalmanFilter(SimpleMatrix P0, LinearizedModel model) {
        MODEL = model;
        this.P = P0;
        this.P_next = P0;
    }



    protected abstract SimpleMatrix getC(SimpleMatrix state, SimpleMatrix input, double t);
    protected abstract SimpleMatrix getD(SimpleMatrix state, SimpleMatrix input, double t);

    protected abstract SimpleMatrix getQ(SimpleMatrix state, SimpleMatrix input, double t);
    protected abstract SimpleMatrix getR(SimpleMatrix state, SimpleMatrix input, double t);



    private SimpleMatrix predict(double dt) {
        // model already has a state, so predict based on that
        xhat_next = MODEL.update();

        ABFTriple abf = MODEL.getLastSystem();
        P_next = abf.getA().mult(P).mult(abf.getA().transpose());

        return xhat_next;
    }

    public SimpleMatrix innovate(SimpleMatrix y) {
        //SimpleMatrix xap = predict(MODEL.getState(), MODEL.getInput());//, t);
        //ABFTriple ABF = MODEL.getDiscreteSystem(t);

        return null;
    }



    public SimpleMatrix getOutput(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return
            // y = Cx + Du
            getC(stateVector, inputVector, t).mult(stateVector)
            .plus(getD(stateVector, inputVector, t).mult(inputVector));
    }
    
}