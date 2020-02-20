package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;

public class LuenbergerObserver extends GenericLuenbergerObserver<LinearizedModel, OutputObserver> {
    private final SimpleMatrix K;

    private SimpleMatrix x_apriori;



    public LuenbergerObserver(StateSpaceSystem<LinearizedModel, OutputObserver> sys, SimpleMatrix K) {
        super(sys);

        this.K = K;
    }



    @Override
    protected SimpleMatrix getK() {
        return K;
    }



    @Override
    protected SimpleMatrix getExpectedOutput() {
        return SYS.getOutput(x_apriori);
    }

    @Override
    public void postApply() {
        x_apriori = SYS.propogate(SYS.getModel().getState());
    }
}