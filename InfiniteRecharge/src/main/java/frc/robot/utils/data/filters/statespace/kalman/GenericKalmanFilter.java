package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.filters.statespace.GenericLuenbergerObserver;

public abstract class GenericKalmanFilter<M extends StateSpaceModel, O extends OutputObserver> extends GenericLuenbergerObserver<M, O> {
    protected SimpleMatrix P;
    protected SimpleMatrix P_apriori;

    protected SimpleMatrix S;
    protected SimpleMatrix Cxy;

    protected SimpleMatrix K;



    protected GenericKalmanFilter(StateSpaceSystem<M, O> sys) {
        super(sys);
    }



    protected abstract SimpleMatrix predictState();
    protected abstract SimpleMatrix predictP();
    protected abstract SimpleMatrix getS();
    protected abstract SimpleMatrix getCxy();
    protected abstract SimpleMatrix updateP();

    public void predict() {
        super.predict();
        
        P_apriori = predictP();
    }

    public void update() {
        S = getS();
        Cxy = getCxy();

        // calculate Kalman gain
        K = Cxy.mult(S.invert());

        P = updateP();
    }

    @Override
    protected SimpleMatrix getK() {
        return K;
    }

    @Override
    public SimpleMatrix calculate(SimpleMatrix output) {
        // calculate Kalman gain
        update();

        // update state given output
        SimpleMatrix x = super.calculate(output);

        SYS.getModel().setState(x);

        return x;
    }
}