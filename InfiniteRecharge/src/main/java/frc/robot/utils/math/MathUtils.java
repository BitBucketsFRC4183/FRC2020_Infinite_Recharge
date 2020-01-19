package frc.robot.utils.math;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;

public class MathUtils {
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
}