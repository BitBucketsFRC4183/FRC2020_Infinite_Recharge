package frc.robot.utils.data.statistics.transforms;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.math.MathUtils;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public abstract class UnscentedTransform {
    public static class SigmaPointsSettings {
        /** Dimension of vector */
        private final int L;

        /** tuning parameter */
        private final double lambda;
        /** weight for Mean estimate of the mean sigma point */
        private final double Wm0;
        /** weight for Covariance estimate of the mean sigma point */
        private final double Wc0;
        /** weights for Mean and Covariance estimates for all non-mean sigma points */
        private final double W;



        /**
         * Create settings to use for determing spread of sigma points
         * 
         * @param alpha tuning parameter that controls spread of sigma points, usually made
         *     really small such as 10^-3
         * @param kappa tuning parameter that controls spread of sigma points, usually set to 0
         * @param beta parameter used to capture some information of the probability distribution
         *     For a Gaussian distribution, 2 is optimal, so this is usually assumed.
         */
        public SigmaPointsSettings(int L, double alpha, double kappa, double beta) {
            this.L = L;

            // calculate lambda
            lambda = alpha * alpha * (L + kappa) - L;
            // we will use 2L+1 sigma points as suggested by Julie and Ulhmann
            // the first is the estimated mean of the distribution and the rest
            // are symmetric around it
            // by adding the mean, it needs a different weight for both mean
            // and covariance calculations, so calculate these weights
            Wm0 = lambda / (lambda + L);
            Wc0 = lambda / (lambda + L) + (1 - alpha * alpha + beta);
            // for any other sigma point that is not the mean, just use the
            // same weight such that 2L*W + Wm0 = 1
            W = 1.0 / (2 * (L + lambda));
        }

        /**
         * Create settings to use for determing spread of sigma points
         * 
         * @param alpha tuning parameter that controls spread of sigma points, usually made
         *     really small such as 10^-3
         * @param kappa tuning parameter that controls spread of sigma points, usually set to 0
         * @param beta parameter used to capture some information of the probability distribution
         *     For a Gaussian distribution, 2 is optimal, so this is usually assumed.
         */
        public SigmaPointsSettings(int L, double alpha, double beta) {
            this(L, alpha, 0, beta);
        }

        public SigmaPointsSettings(int L, double alpha) {
            this(L, alpha, 0, 2);
        }

        public SigmaPointsSettings(int L) {
            this(L, 0.001, 0, 2);
        }

        // return weights
        public double getW() { return W; }
        public double getWm0() { return Wm0; }
        public double getWc0() { return Wc0; }

        /**
         * Generate a set of sigma points to use for an unscented transform
         * 
         * @param x mean of initial distribution
         * @param P covariance of initial distribution
         * @return sigma points distributed around x with covariance P
         */
        public SigmaPoints generateSigmaPoints(SimpleMatrix x, SimpleMatrix P) {
            /** sigma points to use for the Unscented Transform (TM) */
            SimpleMatrix[] sigmaPoints = new SimpleMatrix[2*L + 1];

            // choose sigma points as mean +/- rows of sqrt(P*(L+lambda))
            // where sqrt(P) is a lower triangular matrix A such that AA'=P
            SimpleMatrix xs = MathUtils.chol(P, true).scale(Math.sqrt(L + lambda));
            // a row vector from xs
            SimpleMatrix vec;

            // first sigma point is just the mean
            sigmaPoints[0] = x;
            // calculate the other 2L sigma points
            for (int i = 1; i <= L; i++) {
                // get the i-th row vector from xs
                vec = xs.extractVector(false, i - 1);

                // choose the i-th sigma point
                sigmaPoints[i] = x.plus(vec);
                // choose the (L+i)-th sigma point
                sigmaPoints[L + i] = x.minus(vec);
            }

            return new SigmaPoints(sigmaPoints, this);
        }
    }

    public static class SigmaPoints {
        private final SimpleMatrix[] points;
        private final SigmaPointsSettings settings;

        private SigmaPoints(SimpleMatrix[] points, SigmaPointsSettings settings) {
            this.points = points;
            this.settings = settings;
        }

        public SimpleMatrix[] getPoints() { return points; }
        public SigmaPointsSettings getSettings() { return settings; }
    }

    public static class Distribution {
        private final SimpleMatrix mean;
        private final SimpleMatrix covariance;
        private final SimpleMatrix[] sigmaPoints;
        private final SimpleMatrix[] deviations;

        private Distribution(SimpleMatrix mean, SimpleMatrix covariance, SimpleMatrix[] sigmaPoints, SimpleMatrix[] deviations) {
            this.mean = mean;
            this.covariance = covariance;
            this.sigmaPoints = sigmaPoints;
            this.deviations = deviations;
        }

        public SimpleMatrix getMean() { return mean; }
        public SimpleMatrix getCovariance() { return covariance; }
        public SimpleMatrix[] getSigmaPoints() { return sigmaPoints; }
        public SimpleMatrix[] getDeviations() { return deviations; }
    }



    /**
     * Function to propogate sigma points through to approximate distribution of f
     * given distribution of x
     * 
     * @param x a sigma point
     * @return f(x)
     */
    public abstract SimpleMatrix f(SimpleMatrix x);
    public Distribution approximateDistribution(SimpleMatrix[] sigmas, SigmaPointsSettings settings) {
        SimpleMatrix[] transformed = new SimpleMatrix[sigmas.length];

        for (int i = 0; i < transformed.length; i++) {
            transformed[i] = f(sigmas[i].copy());
        }

        SimpleMatrix mean = transformed[0].scale(settings.getWm0());
        for (int i = 1; i < transformed.length; i++) {
            mean = mean.plus(transformed[i].scale(settings.getW()));
        }

        SimpleMatrix[] es = new SimpleMatrix[transformed.length];
        SimpleMatrix covariance = new SimpleMatrix(mean.numRows(), mean.numCols());

        // loop through all the 2L+1 sigma points
        for (int i = 0; i < transformed.length; i++) {
            // calculate deviation from mean of this sigma point
            es[i] = (transformed[i].minus(mean));

            // add eX*eX' * weight to covariance, but mean point
            // is quirky and has a different mean
            if (i == 0) {
                covariance = es[0].mult(es[0].transpose()).scale(settings.getWc0());
            } else {
                covariance = covariance.plus(es[i].mult(es[i].transpose()).scale(settings.getW()));
            }
        }

        return new Distribution(mean, covariance, transformed, es);
    }

    public Distribution approximateDistribution(SigmaPoints points) {
        return approximateDistribution(points.getPoints(), points.getSettings());
    }
}