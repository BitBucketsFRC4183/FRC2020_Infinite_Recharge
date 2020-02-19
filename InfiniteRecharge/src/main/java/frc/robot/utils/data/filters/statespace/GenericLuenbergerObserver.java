package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.filters.Filter;



public abstract class GenericLuenbergerObserver<T extends StateSpaceModel, O extends OutputObserver> extends Filter<SimpleMatrix> {
    protected final StateSpaceSystem<T, O> SYS;



    public GenericLuenbergerObserver(StateSpaceSystem<T, O> sys) {
        SYS = sys;
    }



    protected abstract SimpleMatrix getExpectedOutput();
    protected abstract SimpleMatrix getK();

    @Override
    public SimpleMatrix calculate(SimpleMatrix output) {
        return SYS.getModel().getState().plus(getK().mult(output.minus(getExpectedOutput())));
    }
}