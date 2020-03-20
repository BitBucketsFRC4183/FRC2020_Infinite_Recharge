package frc.robot.utils.control.statespace.system;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.data.filters.statespace.StateSpaceFilter;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Describes a state space system formed with a model of system dynamics and a
 * model of sensor measurements.
 * 
 * Also has automatic support for StateSpaceFilters. If one is specified, it
 * will automatically update the state given measurements passed through.
 * 
 * @param <M> type of state space model being used to model system dynamics
 * @param <O> type of output observer being used to model sensor outputs
 */
public class StateSpaceSystem<M extends StateSpaceModel, O extends OutputObserver> {
    /** model being used */
    private final M MODEL;
    /** output observer being used */
    private final O OBSERVER;

    /**
     * filter being used, if applicable. will automatically
     * use the filter if it is not null
     */
    private StateSpaceFilter filter;



    /**
     * Create a state space system given a model of the system's dynamics
     * and a model of its sensors
     * 
     * @param model a StateSpaceModel of the system's dynamics
     * @param observer a OutputObserver describing sensor measurements
     */
    public StateSpaceSystem(M model, O observer) {
        this.MODEL = model;
        this.OBSERVER = observer;
    }



    /**
     * Specify a filter for the system to use to take sensor
     * measurements and extrapolate the internal state
     * 
     * @param filter filter to use on the state space model
     */
    public void setFilter(StateSpaceFilter filter) {
        this.filter = filter;
    }



    /**
     * Get the current expected sensor measurement
     * 
     * @return current current expected sensor output,
     *     based on the output observer being used by the
     *     system
     */
    public SimpleMatrix getOutput() {
        return OBSERVER.getOutput(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
    }

    /**
     * Get the expected sensor measurement given a
     * specified state
     * 
     * @param state state to get expected output at
     * @return expected sensor output at the current
     *     time given the specified state
     */
    public SimpleMatrix getOutput(SimpleMatrix state) {
        return OBSERVER.getOutput(state, MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
    }

    /**
     * Propogate a given state to the next time step
     * given the system dynamics
     * 
     * @param state state to propogate from the current time
     *     t[k] to t[k + 1]
     * 
     * @return expected value of specified state at the
     *     next time step, if it were the internal state
     */
    public SimpleMatrix propogate(SimpleMatrix state) {
        return MODEL.propogate(state);
    }

    /**
     * Get the system model being used
     * 
     * @return model of the system's dynamics
     */
    public M getModel() { return MODEL; }

    /**
     * Get the sensor model being used
     * 
     * @return model of the sensors observing the system
     */
    public O getObserver() { return OBSERVER; }


    /**
     * Specify the current input being applied to the system.
     * 
     * If a filter is specified, this may also affect the filter,
     * such as running the prediction step in a Luenberger observer
     * 
     * @param input input being applied to the system
     */
    public void apply(SimpleMatrix input) {
        // move model to next time step and let it internally
        // know the input vector
        MODEL.apply(input);

        // if there is a filter, do what needs to be done after
        // a new input is applied (such as predicting the
        // next state)
        if (filter != null) {
            filter.postApply();
        }
    }

    /**
     * Specify sensor measurements for the filter, if applicable
     * 
     * @param output sensor measurements of the system
     */
    public void observe(SimpleMatrix output) {
        // make sure there is a filter
        if (filter != null) {
            // calculate the internal state of the system given
            // the provided sensor measurements and algorithm
            filter.calculate(output);
        }
    }
}