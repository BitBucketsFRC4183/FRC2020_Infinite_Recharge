package frc.robot.utils.control.statespace.controllers;

import frc.robot.utils.control.statespace.Reference;
import frc.robot.utils.control.statespace.models.StateSpaceModel;

import org.ejml.data.DMatrixRMaj;



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
    public DMatrixRMaj getState() {
        return MODEL.getState();
    }

    /** get the input to apply given an output */
    protected abstract DMatrixRMaj getInput(DMatrixRMaj state);



    /** get the input to apply */
    public DMatrixRMaj getInput() {
        return getInput(getState());
    }
}