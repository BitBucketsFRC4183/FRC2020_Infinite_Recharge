package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ABFTriple;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;



public abstract class UnscentedKalmanFilter {
    private final LinearizedModel MODEL;

    private SimpleMatrix P;
    private SimpleMatrix P_apriori;

    private SimpleMatrix x_apriori;

    private final SimpleMatrix I;



    public UnscentedKalmanFilter(SimpleMatrix P0, LinearizedModel model) {
        MODEL = model;
        this.P = P0;
        this.P_apriori = P0;

        I = SimpleMatrix.identity(model.getNumStates());
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
        x_apriori = MODEL.getState();

        ABFTriple abf = MODEL.getLastSystem();

        SimpleMatrix Q = getQ(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime());
        SimpleMatrix G = getG(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime());

        P_apriori = 
            // APA' + GQG'
            abf.getA().mult(P).mult(abf.getA().transpose()).plus(
                G.mult(Q.mult(G.transpose()))
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
                    getC(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime()).mult(MODEL.getState())
                )
            )
        );

        // posteriori covariance estimate
        P = (I.minus(K.mult(C))).mult(P_apriori);

        MODEL.setState(x_posteriori);

        return P;
    }



    public SimpleMatrix getOutput(SimpleMatrix stateVector, SimpleMatrix inputVector, double t) {
        return
            // y = Cx + Du
            getC(stateVector, inputVector, t).mult(stateVector)
            .plus(getD(stateVector, inputVector, t).mult(inputVector));
    }
    
}