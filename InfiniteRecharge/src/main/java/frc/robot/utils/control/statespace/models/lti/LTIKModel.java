package frc.robot.utils.control.statespace.models.lti;

import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.models.StateSpaceModel;

import org.ejml.simple.SimpleMatrix;



/** LTI model + a constant K, without any dependence of control u on output y */
public abstract class LTIKModel extends StateSpaceModel {
    protected final SimpleMatrix A;
    protected final SimpleMatrix B;
    protected final SimpleMatrix C;
    protected final SimpleMatrix K;

    protected final int STATE_DIM;
    protected final int INPUT_DIM;
    protected final int OUTPUT_DIM;



    public LTIKModel(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix K) throws StateSpaceException {
        if (C == null) {
            C = SimpleMatrix.identity(A.numRows());
        }
        


        STATE_DIM = A.numRows();
        // n x n matrix
        if (A.numCols() != STATE_DIM) {
            throw new StateSpaceException("A must be a square matrix. Got: " + STATE_DIM + "x" + A.numCols());
        }
        this.A = A;



        if (B.numRows() != STATE_DIM) {
            throw new StateSpaceException("B must have " + STATE_DIM + " rows. Got: " + B.numRows() + "x" + B.numCols());
        }
        this.B = B;

        INPUT_DIM = B.numCols();



        if (C.numCols() != STATE_DIM) {
            throw new StateSpaceException("C must have " + STATE_DIM + " columns. Got: " + C.numRows() + "x" + C.numCols());
        }
        OUTPUT_DIM = C.numRows();
        this.C = C;



        if (K.numRows() != STATE_DIM && K.numCols() != 1) {
            throw new StateSpaceException("K must be a " + STATE_DIM + "-vector");
        }
        this.K = K;
    }

    public LTIKModel(SimpleMatrix A, SimpleMatrix B, SimpleMatrix K) throws StateSpaceException {
        this(A, B, SimpleMatrix.identity(A.numRows()), K);
    }

    public LTIKModel(SimpleMatrix[] mats) throws StateSpaceException {
        this(mats[0], mats[1], (mats.length == 4) ? mats[2] : null, (mats.length == 4) ? mats[3] : mats[2]);
    }



    public SimpleMatrix getA() { return A; }
    public SimpleMatrix getB() { return B; }
    public SimpleMatrix getC() { return C; }
    public SimpleMatrix getK() { return K; }
}