package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.Noise;
import frc.robot.utils.math.MathUtils;

/**
 * Unscented Kalman Filters are an extension to Kalman Filters that tend to be more reliable
 * for non-linear systems. The regular Kalman filter assumes that the next state and output are linear
 * combinations of the current state and input. An Extended Kalman Filter linearizes about the
 * current estimates and works with the equations derived for a Kalman Filter.
 * 
 * The Unscented Kalman filter does not assume linearity. The equation used to calculate the Kalman gain
 * is still optimal as long as the mean estimate and the covariance are known exactly.
 * The EKF and UKF differ in how they calculate these. Instead of propogating the mean through the
 * linearized system like an EKF, the UKF keeps track of (2n+1) points in R^n called "sigma points" 
 * where n is the dimension of the states. It is assumed that these points capture the important
 * information of the actual probability distribution (mean and standard deviation). This
 * assumption has been shown to work up to third order, whereas the EKF works up to first.
 * 
 * Some very useful links to learn more:
 *     https://www.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter
 *     https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf
 *     https://en.wikipedia.org/wiki/Unscented_transform
 *     https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter
 */
public class UnscentedKalmanFilter extends GenericKalmanFilter<StateSpaceModel, OutputObserver> {
    /** tuning parameter */
    private final double lambda;
    /** weight for Mean estimate of the mean sigma point */
    private final double Wm0;
    /** weight for Covariance estimate of the mean sigma point */
    private final double Wc0;
    /** weights for Mean and Covariance estimates for all non-mean sigma points */
    private final double W;

    /** sigma points to use for the Unscented Transform (TM) */
    private SimpleMatrix[] sigmaPoints;

    /** deviations in apriori state from mean */
    private SimpleMatrix[] eX;
    /** deviations in apriori output from mean */
    private SimpleMatrix[] eY;

    /** average of transformed sigma points, or expected a priori output */
    private SimpleMatrix yAvg;



    /**
     * Create an Unscented Kalman Filter for a system
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha tuning parameter that controls spread of sigma points, usually made
     *     really small such as 10^-3
     * @param kappa tuning parameter that controls spread of sigma points, usually set to 0
     * @param beta parameter used to capture some information of the probability distribution
     *     For a Gaussian distribution, 2 is optimal, so this is usually assumed.
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha, double kappa, double beta) {
        super(sys);



        // initialize the estimate covariance
        P = P0;

        // get the number of states
        int L = sys.getModel().getNumStates();

        // calculate lambda
        lambda = alpha * alpha * (L + kappa) - L;
        // we will use 2L+1 sigma points as suggested by Julie and Ulhmann
        // the first is the estimated mean of the distribution and the rest
        // are symmetric around it
        // by adding the mean, it needs a different weight for both mean
        // and covariance calculations, so calculate these weights
        Wm0 = lambda / (lambda + L);
        Wc0 = lambda / (lambda + L) + (1 - alpha * alpha + beta * beta);
        // for any other sigma point that is not the mean, just use the
        // same weight such that 2L*W + Wm0 = 1
        W = 1.0 / (2 * (L + lambda));

        // initialize the set of sigma points
        sigmaPoints = new SimpleMatrix[2*L + 1];
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa is defaulted to 0
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha tuning parameter that controls spread of sigma points, usually made
     *     really small such as 10^-3
     * @param beta parameter used to capture some information of the probability distribution
     *     For a Gaussian distribution, 2 is optimal, so this is usually assumed.
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha, double beta) {
        this(sys, P0, alpha, 0, beta);
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa and beta are defaulted to 0 and 2 respectively
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha tuning parameter that controls spread of sigma points, usually made
     *     really small such as 10^-3
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha) {
        this(sys, P0, alpha, 0, 2);
    }

    /**
     * Create an Unscented Kalman Filter for a system. alpha, kappa, and beta are defaulted to 10^-3,
     *     0, and 2 respectively
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0) {
        this(sys, P0, Math.pow(10, -3), 0, 2);
    }



    @Override
    public void predict() {
        // sigma points are used for predict step so calculate them beforehand
        calculateSigmaPoints();

        super.predict();
    }



    /**
     * Calculate the sigma points used for the Unscented Transform, to calculate
     * a priori state mean and covariance and output
     */
    private void calculateSigmaPoints() {
        StateSpaceModel model = SYS.getModel();

        // first one is the mean, or expected value, of the state, which is
        // already known
        sigmaPoints[0] = model.getState();
        // propogate it to the next state
        sigmaPoints[0] = model.propogate(sigmaPoints[0]);
        // other sigma points require some linear algebra to find, so that
        // is done a few lines below

        // x_apriori is a (weighted) mean of all transformed sigma points
        // so just start with the first one since we already know the value
        x_apriori = sigmaPoints[0].scale(Wm0);

        // number of states in the system
        int L = model.getNumStates();

        // choose sigma points as mean +/- rows of sqrt(P*(L+lambda))
        // where sqrt(P) is a lower triangular matrix A such that AA'=P
        SimpleMatrix xs = MathUtils.chol(P, true).scale(Math.sqrt(L + lambda));
        // a row vector from xs
        SimpleMatrix vec;

        /** calculate the other 2L sigma points */
        for (int i = 1; i <= L; i++) {
            // get the i-th row vector from xs
            vec = xs.extractVector(true, i);

            // choose the i-th sigma point
            sigmaPoints[i] = sigmaPoints[0].plus(vec);
            // and propogate it to the next time step
            sigmaPoints[i] = model.propogate(sigmaPoints[i]);

            // choose the (L+i)-th sigma point
            sigmaPoints[L + i] = sigmaPoints[0].minus(vec);
            // also propogate it to the next time step
            sigmaPoints[L + i] = model.propogate(sigmaPoints[L + i]);
        }

        // now the sigma points have all been generated and transformed
    }



    @Override
    protected SimpleMatrix predictState() {
        StateSpaceModel model = SYS.getModel();
        int L = model.getNumStates();



        // a priori state is a weighted sum of transformed sigma points
        // the 0th sigma point is quirky and has a different weight so
        // just start with that and for-loop in the rest
        SimpleMatrix x = sigmaPoints[0].scale(Wm0);

        // add other weighted sigma points
        for (int i = 1; i <= 2*L; i++) {
            x = x.plus(sigmaPoints[i].scale(W));
        }

        // at this point "x" is the a priori estimate

        return x;
    }

    @Override
    protected SimpleMatrix predictP() {
        StateSpaceModel model = SYS.getModel();
        int L = model.getNumStates();

        // calculate state covariance empirically as a weighted average of
        // square errors of just the sigma points, assuming they are
        // representative of the entire distribution

        // deviations of sigma points from mean
        eX = new SimpleMatrix[2*L + 1];
        // covariance of x to return
        SimpleMatrix p = new SimpleMatrix(L, L);

        // loop through all the 2L+1 sigma points
        for (int i = 0; i <= 2*L; i++) {
            // calculate deviation from mean of this sigma point
            eX[i] = (sigmaPoints[i].minus(x_apriori));

            // add eX*eX' * weight to covariance, but mean point
            // is quirky and has a different mean
            if (i == 0) {
                p = eX[0].mult(eX[0].transpose()).scale(Wc0);
            } else {
                p = p.plus(eX[i].mult(eX[i].transpose()).scale(W));
            }
        }

        Noise processNoise = model.getNoiseSource().getNoise(
            model.getState(),
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        // add in the process noise's covariance because real world
        // systems b like that
        return p.plus(processNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getS() {
        StateSpaceModel model = SYS.getModel();
        OutputObserver outputObserver = SYS.getObserver();
        int L = model.getNumStates();

        // calculate covariance of a priori output estimate by
        // getting outputs of the transformed sigma points
        // and just finding their covariance empirically, assuming
        // they are representative of the entire distribution

        // outputs of transformed sigma points
        SimpleMatrix[] ys = new SimpleMatrix[2*L + 1];

        // already calculated transformed sigma points, just get the output
        for (int i = 0; i <= 2*L; i++) {
            ys[i] = outputObserver.getOutput(sigmaPoints[0], model.getInput(), model.getLastTime(), model.getCount());
        }

        // a priori/mean estimate of output
        // mean is quirky and has a different weight, so start with it
        yAvg = ys[0].scale(Wc0);
        for (int i = 1; i <= 2*L; i++) {
            // add in the rest of the weighted means to get average
            yAvg = yAvg.plus(ys[i].scale(W));
        }

        // deviations in y from a priori output yAvg
        eY = new SimpleMatrix[2*L + 1];
        for (int i = 0; i <= 2*L; i++) {
            // just calculate the deviation
            eY[i] = ys[i].minus(yAvg);
        }

        // find covariance of eY by adding eY*eY' (weighted)
        // again the mean just b like that so start with mean
        SimpleMatrix s = eY[0].mult(eY[0].transpose()).scale(Wc0);
        for (int i = 1; i <= 2*L; i++) {
            S = S.plus(eY[i].mult(eY[i].transpose()).scale(W));
        }

        Noise outputNoise = outputObserver.getNoiseSource().getNoise(
            x_apriori,
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        // there's always some noise from the output so just add
        // its covariance
        return s.plus(outputNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getCxy() {
        StateSpaceModel model = SYS.getModel();
        OutputObserver outputObserver = SYS.getObserver();
        int L = model.getNumStates();

        // X by Y
        // matrix to put cross-variance between X and Y into
        SimpleMatrix C = new SimpleMatrix(model.getNumStates(), outputObserver.getNumOutputs());
        // cross variance just between one sigma point and its output
        SimpleMatrix cross;

        for (int i = 0; i <= 2*L; i++) {
            // calculate this sigma point's individual cross variance
            cross = eX[i].mult(eY[i].transpose());

            // add it to the bunch
            if (i == 0) {
                C = cross.scale(Wc0);
            } else {
                C = C.plus(cross.scale(W));
            }
        }

        return C;
    }

    @Override
    protected SimpleMatrix updateP() {
        return P_apriori.minus(K.mult(S).mult(K.transpose()));
    }

    @Override
    protected SimpleMatrix getExpectedOutput() {
        return yAvg;
    }
}