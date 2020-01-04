package frc.robot.utils.control.statespace.models.lti;

import frc.robot.utils.control.statespace.StateSpaceException;

import org.ejml.data.DMatrixRMaj;



public abstract class LTIModel extends LTIKModel {
    public LTIModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C) throws StateSpaceException {
        super(A, B, C, (new DMatrixRMaj(A.getNumRows(), 1)));
    }

    public LTIModel(DMatrixRMaj A, DMatrixRMaj B) throws StateSpaceException {
        super(A, B, (new DMatrixRMaj(A.getNumRows(), 1)));
    }
}