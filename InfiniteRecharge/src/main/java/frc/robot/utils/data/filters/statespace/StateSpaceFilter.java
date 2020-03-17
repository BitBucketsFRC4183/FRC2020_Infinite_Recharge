package frc.robot.utils.data.filters.statespace;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.data.filters.Filter;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Filter that deals with extrapolating the internal state of a
 * state space system given (possibly noisy) sensor data
 */
public abstract class StateSpaceFilter extends Filter<SimpleMatrix> {
    /**
     * Specify a routine to do once an input to a system has been
     * updated and it is moved to the next time step.
     */
    public abstract void postApply();



    @Override
    /**
     * Update the estimated internal state of the system given
     * sensor measurements
     * 
     * @param measurement sensor measurements to update the
     *     state estimate with
     */
    public abstract SimpleMatrix calculate(SimpleMatrix measurement);
}