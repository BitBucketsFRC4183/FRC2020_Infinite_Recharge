package frc.robot.utils.math.interpolation.spline;

import java.util.Arrays;

import frc.robot.utils.math.Polynomial;

// spline for interpolation between known data points
public class Spline {
    private final Polynomial[] polys;
    private final double[] xs;
    private final int n;

    /**
     * Create a spline that interpolates between a set of given points
     * 
     * @param x set of x inputs (must be increasing!)
     * @param y set of y outputs
     * 
     * @return a spline that goes through every (x, y) and interpolates between them
     */
    public Spline(double[] x, double[] y) {
        if (x.length != y.length) {
            throw new IllegalArgumentException("Array x must be the same size as y");
        }

        if (x.length < 2) {
            throw new IllegalArgumentException("Cannot interpolate between less than two points with a spline");
        }

        // yes this is copied straight from Wikipedia:
        // https://en.wikipedia.org/wiki/Spline_(mathematics)#Definition
        // it's build season nobody got time for learning stuff
        n = x.length - 1;

        double[] a = new double[n + 1];
        for (int i = 0; i < n + 1; i++) {
            a[i] = y[i];
        }
        
        double[] b = new double[n];
        double[] d = new double[n];

        double[] h = new double[n];
        for (int i = 0; i < n; i++) {
            h[i] = x[i + 1] - x[i];

            if (h[i] <= 0) {
                throw new IllegalArgumentException("Array x must be strictly increasing");
            }
        }

        double[] alpha = new double[n]; // the Algorithm(TM) wants the index to start at 1 but we can't
        for (int i = 1; i <= n - 1; i++) {
            alpha[i - 1] = 3/h[i] * (a[i + 1] - a[i]) - 3/h[i - 1] * (a[i] - a[i - 1]);
        }

        double[] c = new double[n + 1];
        double[] l = new double[n + 1];
        double[] mu = new double[n + 1];
        double[] z = new double[n + 1];

        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;

        for (int i = 1; i < n - 1; i++) {
            l[i] = 2*(x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i + 1] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 1;
        z[n] = 0;
        c[n] = 0;

        for (int j = n - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2*c[j])/3;
            d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
        }

        polys = new Polynomial[n];
        for (int i = 0; i < n; i++) {
            polys[i] = new Polynomial(new double[] {a[i], b[i], c[i], d[i]});
        }

        xs = x;
    }

    public double get(double x) {
        if (x < xs[0]) {
            return get(xs[0]);
        }

        if (x > xs[n]) {
            return get(xs[n]);
        }

        int index = Arrays.binarySearch(xs, x);
        // from documentation, point is either > 0 if x is in xs or < 0 but equal to -(insertion point + 1)
        int beforeIndex;
        if (index >= 0 && index < n) {
            beforeIndex = index;
        } else if (index == n) {
            beforeIndex = n - 1; // last index
        } else {
            beforeIndex = -index - 2; // want element before insertionPoint
        }

        return polys[beforeIndex].eval(x - xs[beforeIndex]);
    }

    public Polynomial[] getPolynomials() { return polys; }
}