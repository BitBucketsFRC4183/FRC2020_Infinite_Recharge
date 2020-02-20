package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;



public abstract class GenericLuenbergerObserver<T extends StateSpaceModel, O extends OutputObserver> extends StateSpaceFilter {
    protected final StateSpaceSystem<T, O> SYS;

    protected SimpleMatrix x_apriori;



    public GenericLuenbergerObserver(StateSpaceSystem<T, O> sys) {
        SYS = sys;
    }



    protected abstract SimpleMatrix predictState();
    public void predict() {
        x_apriori = predictState();
    }

    protected abstract SimpleMatrix getExpectedOutput();
    protected abstract SimpleMatrix getK();

    @Override
    public SimpleMatrix calculate(SimpleMatrix output) {
        return x_apriori.plus(getK().mult(output.minus(getExpectedOutput())));
    }

    @Override
    public void postApply() {
        predict();
    }
}