package frc.robot.utils.control.statespace.models.lti;

import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.models.StateSpaceModel;

import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.data.DMatrixRMaj;



/** LTI model + a constant K, without any dependence of control u on output y */
public abstract class LTIKModel extends StateSpaceModel {
    protected final DMatrixRMaj A;
    protected final DMatrixRMaj B;
    protected final DMatrixRMaj C;
    protected final DMatrixRMaj K;

    protected final int STATE_DIM;
    protected final int INPUT_DIM;
    protected final int OUTPUT_DIM;



    public LTIKModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj K) throws StateSpaceException {
        STATE_DIM = A.getNumRows();
        // n x n matrix
        if (A.getNumCols() != STATE_DIM) {
            throw new StateSpaceException("A must be a square matrix. Got: " + STATE_DIM + "x" + A.getNumCols());
        }
        this.A = A;



        if (B.getNumRows() != STATE_DIM) {
            throw new StateSpaceException("B must have " + STATE_DIM + " rows. Got: " + B.getNumRows() + "x" + B.getNumCols());
        }
        this.B = B;

        INPUT_DIM = B.getNumCols();



        if (C.getNumCols() != STATE_DIM) {
            throw new StateSpaceException("C must have " + STATE_DIM + " columns. Got: " + C.getNumRows() + "x" + C.getNumCols());
        }
        OUTPUT_DIM = C.getNumRows();
        this.C = C;



        if (K.getNumRows() != STATE_DIM && K.getNumCols() != 1) {
            throw new StateSpaceException("K must be a " + STATE_DIM + "-vector");
        }
        this.K = K;
    }

    public LTIKModel(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj K) throws StateSpaceException {
        this(A, B, CommonOps_DDRM.identity(A.getNumRows()), K);
    }



    public DMatrixRMaj getA() { return A; }
    public DMatrixRMaj getB() { return B; }
    public DMatrixRMaj getC() { return C; }
    public DMatrixRMaj getK() { return K; }



    public abstract DMatrixRMaj getOutput();
}