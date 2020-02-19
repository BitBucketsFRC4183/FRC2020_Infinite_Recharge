package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.filters.statespace.GenericLuenbergerObserver;

public abstract class GenericKalmanFilter<M extends StateSpaceModel, O extends OutputObserver> extends GenericLuenbergerObserver<M, O> {
    protected SimpleMatrix x_apriori;

    protected SimpleMatrix P;
    protected SimpleMatrix P_apriori;

    protected SimpleMatrix S;
    protected SimpleMatrix Cxy;

    protected SimpleMatrix K;



    protected GenericKalmanFilter(StateSpaceSystem<M, O> sys) {
        super(sys);
    }

    @Override
    protected SimpleMatrix getExpectedOutput() {
        return x_apriori;
    }



    protected abstract SimpleMatrix predictState();
    protected abstract SimpleMatrix predictP();
    protected abstract SimpleMatrix getS();
    protected abstract SimpleMatrix getCxy();
    protected abstract SimpleMatrix updateP();

    protected void predict() {
        x_apriori = predictState();
        P_apriori = predictP();
    }

    protected void update() {
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
        return super.calculate(output);
    }
}