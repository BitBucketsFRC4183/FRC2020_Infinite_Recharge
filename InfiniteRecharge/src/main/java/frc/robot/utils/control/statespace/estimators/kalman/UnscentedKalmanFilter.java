package frc.robot.utils.control.statespace.estimators.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.models.linearized.LinearizedModel;
import frc.robot.utils.math.MathUtils;



/**
 * Unscented Kalman Filters are an extension to Kalman Filters that tend to be more reliable
 * for non-linear systems. The Kalman filter assumes that the next state and output are linear
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
public abstract class UnscentedKalmanFilter {
    private final StateSpaceModel MODEL;

    private SimpleMatrix P;
    private SimpleMatrix P_apriori;

    private SimpleMatrix x_apriori;

    private final SimpleMatrix I;



    private final double lambda;
    // weight for Mean estimate of the mean sigma point
    private final double Wm0;
    // weight for Covariance estimate of the mean sigma point
    private final double Wc0;
    // weights for Mean and Covariance estimates for all non-mean sigma points
    private final double W;

    private SimpleMatrix[] sigmaPoints;

    // deviations in apriori state and output from mean
    private SimpleMatrix[] eX;
    private SimpleMatrix[] eY;



    public UnscentedKalmanFilter(SimpleMatrix P0, StateSpaceModel model, double alpha, double kappa, double beta) {
        MODEL = model;
        this.P = P0;
        this.P_apriori = P0;

        I = SimpleMatrix.identity(model.getNumStates());



        int L = model.getNumStates();

        this.lambda = alpha * alpha * (L + kappa) - L;
        Wm0 = lambda / (lambda + L);
        Wc0 = lambda / (lambda + L) + (1 - alpha * alpha + beta * beta);
        W = 1.0 / (2 * (L + lambda));

        sigmaPoints = new SimpleMatrix[2*L + 1];
    }

    public UnscentedKalmanFilter(SimpleMatrix P0, LinearizedModel model, double alpha, double beta) {
        this(P0, model, alpha, 0.0, beta);
    }

    public UnscentedKalmanFilter(SimpleMatrix P0, LinearizedModel model, double alpha) {
        this(P0, model, alpha, 0.0, 2);
    }

    public UnscentedKalmanFilter(SimpleMatrix P0, LinearizedModel model) {
        this(P0, model, Math.pow(10, -3), 0.0, 2.0);
    }



    protected abstract SimpleMatrix getOutput(SimpleMatrix state, SimpleMatrix input, double t, int k);

    protected abstract SimpleMatrix getQ(SimpleMatrix state, SimpleMatrix input, double t, int k);
    protected abstract SimpleMatrix getR(SimpleMatrix state, SimpleMatrix input, double t, int k);
    protected abstract SimpleMatrix getG(SimpleMatrix state, SimpleMatrix input, double t, int k);



    /**
     * 
     * @param dt
     * @return
     */
    public SimpleMatrix predict() {
        sigmaPoints[0] = MODEL.getState();
        sigmaPoints[0] = MODEL.propogate(sigmaPoints[0]);

        int L = MODEL.getNumStates();

        SimpleMatrix xs = MathUtils.chol(P, true).scale(Math.sqrt(L + lambda));
        SimpleMatrix vec;

        x_apriori = sigmaPoints[0].scale(Wm0);

        for (int i = 1; i <= L; i++) {
            vec = xs.extractVector(true, i);

            sigmaPoints[i] = sigmaPoints[0].plus(vec);
            sigmaPoints[i] = MODEL.propogate(sigmaPoints[i]);

            sigmaPoints[L + i] = sigmaPoints[0].minus(vec);
            sigmaPoints[L + i] = MODEL.propogate(sigmaPoints[L + i]);

            x_apriori = x_apriori.plus(sigmaPoints[i].scale(W));
            x_apriori = x_apriori.plus(sigmaPoints[L + i].scale(W));
        }

        eX = new SimpleMatrix[2*L + 1];

        for (int i = 0; i < 2 * L + 1; i++) {
            eX[i] = (sigmaPoints[i].minus(x_apriori));

            if (i == 0) {
                P_apriori = eX[0].mult(eX[0].transpose()).scale(Wc0);
            } else {
                P_apriori = P_apriori.plus(eX[i].mult(eX[i].transpose()).scale(W));
            }
        }

        SimpleMatrix Q = getQ(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
        SimpleMatrix G = getG(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());

        P_apriori = P_apriori.plus(G.mult(Q.mult(G.transpose())));

        return x_apriori;
    }

    public SimpleMatrix update(SimpleMatrix y) {
        int L = MODEL.getNumStates();

        SimpleMatrix[] ys = new SimpleMatrix[2*L + 1];

        // already calculated transformed sigma points
        for (int i = 0; i < 2*L + 1; i++) {
            ys[i] = getOutput(sigmaPoints[0], MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
        }

        SimpleMatrix yAvg = ys[0].scale(Wm0);
        for (int i = 1; i < 2*L + 1; i++) {
            yAvg = yAvg.plus(ys[i].scale(W));
        }

        eY = new SimpleMatrix[2*L + 1];
        for (int i = 0; i < 2*L + 1; i++) {
            eY[i] = ys[i].minus(yAvg);
        }

        SimpleMatrix S = eY[0].mult(eY[0].transpose()).scale(Wc0);
        for (int i = 1; i < 2*L + 1; i++) {
            S = S.plus(eY[i].mult(eY[i].transpose()).scale(W));
        }

        SimpleMatrix R = getR(MODEL.getState(), MODEL.getInput(), MODEL.getLastTime(), MODEL.getCount());
        S = S.plus(R);



        // X by Y
        SimpleMatrix C = new SimpleMatrix(MODEL.getNumStates(), yAvg.numRows());
        SimpleMatrix cross;

        for (int i = 0; i < 2*L + 1; i++) {
            cross = eX[i].mult(eY[i].transpose());

            if (i == 0) {
                C = C.plus(cross.scale(Wc0));
            } else {
                C = C.plus(cross.scale(W));
            }
        }



        SimpleMatrix K = C.mult(S.invert());

        SimpleMatrix x_posteriori = x_apriori.plus(K.mult(y.minus(yAvg)));

        // posteriori covariance estimate
        P = (I.minus(K.mult(C))).mult(P_apriori);

        MODEL.setState(x_posteriori);

        return P;
    }
}