package frc.robot.utils.math;

public class MathUtils {
	public static double unitConverter(double quantity, double unitToBeConverted, double intoThisOne) {
		return quantity * (intoThisOne / unitToBeConverted);
	}

	public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
		return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	}
}