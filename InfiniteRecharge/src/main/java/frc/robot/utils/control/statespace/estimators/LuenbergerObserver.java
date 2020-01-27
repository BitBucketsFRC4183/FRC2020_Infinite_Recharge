package frc.robot.utils.control.statespace.estimators;

import org.ejml.simple.SimpleMatrix;



public abstract class LuenbergerObserver {
    private SimpleMatrix state;
    private SimpleMatrix output;

    private SimpleMatrix K;



    public LuenbergerObserver(SimpleMatrix state) {
        this.state = state;
    }



    public void setK(SimpleMatrix K) {
        this.K = K;
    }



    public abstract SimpleMatrix aprioriUpdate(SimpleMatrix state);
    public abstract SimpleMatrix getOutput(SimpleMatrix state);

    public SimpleMatrix update(SimpleMatrix newOutput) {
        // post x_k = x_k + K(y_k - f(x_k'))
        return state.plus(K.mult(newOutput.minus(getOutput(output))));
    }
}