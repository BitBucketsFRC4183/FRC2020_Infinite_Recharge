package frc.robot.utils.control.statespace.models.lti.estimators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.models.lti.LTIKModel;



public abstract class LTIEstimatorModel extends LTIKModel {
    public LTIEstimatorModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj L) throws StateSpaceException {
        super(A, B, C, new DMatrixRMaj(A.getNumRows(), 1));
    }

    public LTIEstimatorModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj L) throws StateSpaceException {
        this(A, B, CommonOps_DDRM.identity(A.getNumRows()), L);
    }
}