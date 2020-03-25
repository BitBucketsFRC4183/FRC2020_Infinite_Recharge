package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * (or alternatively)
 * Don’t ignore this file. Look upon
 * my mighty generics and cower!
 * ==================================
 */
/**
 * Class that describes any Luenberger observer. A Kalman filter is considered
 * a type of Luenberger observer, just with a very carefully chosen gain
 * matrix, but the Luenberger observer is more general.
 * 
 * @param <M> type of state space model being filtered
 * @param <O> type of output observer being used for sensor measurements
 */
public abstract class GenericLuenbergerObserver<M extends StateSpaceModel, O extends OutputObserver> extends StateSpaceFilter {
    // the state space system being filtered
    protected final StateSpaceSystem<M, O> SYS;

    // a priori estimate of state after an input is applied
    protected SimpleMatrix x_apriori;



    /**
     * Create a Luenberger observer for a given state space system
     * 
     * @param sys system to filter with a Luenberger observer
     */
    public GenericLuenbergerObserver(StateSpaceSystem<M, O> sys) {
        SYS = sys;
    }



    /**
     * Predict the next state given the current state (or an
     * estimate thereof) and the current input are known.
     * 
     * @return a priori estimate of the next state
     */
    protected abstract SimpleMatrix predictState();
    /**
     * Prediction step of a Luenberger observer
     */
    public void predict() {
        // just update the a priori state estimate
        // the Kalman filter adds more things to the predict
        // step
        x_apriori = predictState();
    }

    /**
     * Get the expected output of the system given the filter
     * knows the a priori state estimate
     * 
     * @return expected output of the system
     */
    protected abstract SimpleMatrix getExpectedOutput();
    /**
     * Get the current gain being used to correct state based on
     *     deviations from sensor measurements
     * 
     * @return gain being used
     */
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