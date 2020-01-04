package frc.robot.utils.control.statespace.models;

import org.ejml.data.DMatrixRMaj;



public abstract class StateSpaceModel {
    public abstract DMatrixRMaj getState();
}