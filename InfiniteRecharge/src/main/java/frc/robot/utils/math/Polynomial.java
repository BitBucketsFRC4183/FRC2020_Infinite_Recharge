package frc.robot.utils.math;

public class Polynomial {
    private double[] coeffs;

    public Polynomial(double[] coeffs) {
        this.coeffs = coeffs;
    }

    public double[] getCoefficients() {
        return coeffs;
    }

    public Polynomial deriv() {
        double[] coeffs2 = new double[coeffs.length - 1];

        // coeffs[n + 1] x^(n + 1) -> (n + 1) coeffs[n + 1] x^n
        for (int i = 0; i < coeffs2.length; i++) {
            coeffs[i] = coeffs[i + 1] * (i + 1);
        }

        return new Polynomial(coeffs2);
    }

    public double eval(double x) {
        double sum = 0;

        for (int i = 0; i < coeffs.length; i++) {
            sum += coeffs[i] * Math.pow(x, i);
        }

        return sum;
    }
}