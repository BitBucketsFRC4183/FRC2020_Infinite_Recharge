package frc.robot.utils.control.statespace.controllers;

import frc.robot.utils.control.statespace.Reference;
import frc.robot.utils.control.statespace.models.StateSpaceModel;

import org.ejml.simple.SimpleMatrix;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class StateSpaceController {
    protected final StateSpaceModel MODEL;
    protected Reference ref;



    public StateSpaceController(StateSpaceModel model) {
        MODEL = model;
    }



    public void setReference(Reference ref) {
        this.ref = ref;
    }



    /** get the state of the system */
    public abstract SimpleMatrix getState();

    /** get the input to apply given an output */
    protected abstract SimpleMatrix getInput(SimpleMatrix state);



    /** get the input to apply */
    public SimpleMatrix getInput() {
        return getInput(getState());
    }
}