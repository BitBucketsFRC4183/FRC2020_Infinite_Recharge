package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Luenberger observer for a system with linearized dynamics
 */
public class LuenbergerObserver extends GenericLuenbergerObserver<LinearizedModel, OutputObserver> {
    // assume constant gain
    private final SimpleMatrix K;



    /**
     * Create a Luenberger observer for a linearized system with a constant gain
     * 
     * @param sys linearized system to filter
     * @param K gain to use to correct deviations from sensor measurements
     */
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
    protected SimpleMatrix predictState() {
        return SYS.propogate(SYS.getModel().getState());
    }
}