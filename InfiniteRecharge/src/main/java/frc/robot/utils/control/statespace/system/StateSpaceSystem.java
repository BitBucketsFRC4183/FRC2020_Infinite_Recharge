package frc.robot.utils.control.statespace.system;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;

public class StateSpaceSystem<M extends StateSpaceModel, O extends OutputObserver> {
    private final M MODEL;
    private final O OBSERVER;



    public StateSpaceSystem(M model, O observer) {
        this.MODEL = model;
        this.OBSERVER = observer;
    }



    public SimpleMatrix getOutput() {
        return OBSERVER.getOutput(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
    }

    public SimpleMatrix getOutput(SimpleMatrix state) {
        return OBSERVER.getOutput(state, MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
    }

    public SimpleMatrix propogate(SimpleMatrix state) {
        return MODEL.propogate(state);
    }

    public M getModel() { return MODEL; }
    public O getObserver() { return OBSERVER; }
}