package frc.robot.utils.math;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;

public class MathUtils {

	public static final double G = 32.17405;

	public static double unitConverter(double quantity, double unitToBeConverted, double intoThisOne) {
		return quantity * (intoThisOne / unitToBeConverted);
	}

	public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
		return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	}



	// put in here in case we ever need to change the algorithm
	/**
	 * Compute the matrix exponential, e^M of the given matrix.
	 *
	 * @param matrix The matrix to compute the exponential of.
	 * @return The resultant matrix.
	 */
	public static SimpleMatrix expm(SimpleMatrix matrix) {
		return SimpleMatrixUtils.expm(matrix);
    }
    


    /**
     * Cholesky decomposition of a matrix
     */
    // mayhaps got from https://ejml.org/wiki/index.php?title=Matrix_Decompositions#SimpleMatrix
    public static SimpleMatrix chol(SimpleMatrix matrix, boolean lower) {
        CholeskyDecomposition_F64<DMatrixRMaj> decomp = DecompositionFactory_DDRM.chol(matrix.numRows(), lower);

        if (!decomp.decompose(matrix.getMatrix())) {
            return null; // and hope it doesn't die
        }

        return SimpleMatrix.wrap(decomp.getT(null));
    }
}