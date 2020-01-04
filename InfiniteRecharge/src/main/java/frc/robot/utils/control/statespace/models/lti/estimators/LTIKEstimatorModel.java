package frc.robot.utils.control.statespace.models.lti.estimators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.models.lti.LTIKModel;



public abstract class LTIKEstimatorModel extends LTIKModel {
    private final DMatrixRMaj L;

    private DMatrixRMaj stateEstimate;
    private DMatrixRMaj input;
    private DMatrixRMaj output;



    public LTIKEstimatorModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj K, DMatrixRMaj L) throws StateSpaceException {
        super(A, B, C, K);

        // A-LC is a n x n matrix
        // LC is n x n
        // C is outputDim x n
        // thus L must be n x outputDim

        if (L.getNumRows() != STATE_DIM) {
            throw new StateSpaceException("L must have " + STATE_DIM + " rows. Got: " + L.getNumRows() + "x" + L.getNumCols());
        }

        if (L.getNumCols() != OUTPUT_DIM) {
            throw new StateSpaceException("L must have " + OUTPUT_DIM + " columns. Got: " + L.getNumRows() + "x" + L.getNumCols());
        }

        this.L = L;



        stateEstimate = new DMatrixRMaj(STATE_DIM, 1);
        input = new DMatrixRMaj(INPUT_DIM, 1);
        output = new DMatrixRMaj(OUTPUT_DIM, 1);
    }

    public LTIKEstimatorModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj K, DMatrixRMaj L) throws StateSpaceException {
        this(A, B, CommonOps_DDRM.identity(A.getNumRows()), K, L);
    }





    public void init(DMatrixRMaj state0) {
        if (state0.getNumRows() == STATE_DIM && state0.getNumCols() == 1) {
            stateEstimate = state0;
        }
    }



    /* Do this BEFORE calling update */
    public void setOutput(DMatrixRMaj output) {
        if (output.getNumRows() == OUTPUT_DIM && output.getNumCols() == 1) {
            this.output = output;
        }
    }

    /* Do this BEFORE calling update */
    public void setInput(DMatrixRMaj input) {
        if (input.getNumRows() == INPUT_DIM && input.getNumCols() == 1) {
            this.input = input;
        }
    }



    public DMatrixRMaj update() {
        // output should be that of last iteration

        DMatrixRMaj outputEstimate = null;
        DMatrixRMaj outputError = null;
        CommonOps_DDRM.mult(C, stateEstimate, outputEstimate);
        CommonOps_DDRM.subtract(output, outputEstimate, outputError);



        CommonOps_DDRM.multAdd(A, stateEstimate, stateEstimate);
        CommonOps_DDRM.multAdd(B, input, stateEstimate);
        CommonOps_DDRM.add(stateEstimate, K, stateEstimate);
        CommonOps_DDRM.multAdd(L, outputError, stateEstimate);

        return stateEstimate;
    }



    @Override
    public DMatrixRMaj getState() {
        return stateEstimate;
    }
}