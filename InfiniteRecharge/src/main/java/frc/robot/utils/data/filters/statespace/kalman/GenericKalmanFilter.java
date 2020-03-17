package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.filters.statespace.GenericLuenbergerObserver;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Class to represent any type of Kalman filter.
 * The Kalman filter is a type of Luenberger observer with a changing gain
 * that (theoretically) best estimates the internal state of a system
 * 
 * @param <M> type of model the filter is being used on
 * @param <O> output observer on the system
 */
public abstract class GenericKalmanFilter<M extends StateSpaceModel, O extends OutputObserver> extends GenericLuenbergerObserver<M, O> {
    /** true covariance in state estimate */
    protected SimpleMatrix P;
    /**
     * estimated covariance in state estimate based on the transformation
     * the state goes through
     */ 
    protected SimpleMatrix P_apriori;

    /** estimated covariance of expected output of the system */
    protected SimpleMatrix S;
    /** cross-variance between expected state and output of the system */
    protected SimpleMatrix Cxy;

    /** current Kalman gain being used (changes!) */
    protected SimpleMatrix K;



    protected GenericKalmanFilter(StateSpaceSystem<M, O> sys) {
        super(sys);
    }



    /**
     * Predict the covariance (aka get the a priori covariance) in
     * the state estimate by transforming the previous true covariance
     * 
     * @return a priori covariance of state in the next time step
     */
    protected abstract SimpleMatrix predictP();
    /**
     * Estimate the covariance in the expected (a priori) output
     * 
     * @return covariance of output next time step
     */
    protected abstract SimpleMatrix getS();
    /**
     * Estimate the cross-variance between a priori state and output
     * 
     * @return cross-variance between a priori state and output
     */
    protected abstract SimpleMatrix getCxy();
    /**
     * Update the covariance of the state estimate once new measurements
     * have been provided
     * 
     * @return new (posteriori) covariance of the state estimate
     */
    protected abstract SimpleMatrix updateP();

    @Override
    public void predict() {
        super.predict();
        
        P_apriori = predictP();
    }

    /**
     * Update step of the Kalman filter, where the new Kalman
     * gain is calculated. Some authors may group calculating the
     * new state estimate into the update step, but that is done
     * in the calculate method in this implemenation
     */
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