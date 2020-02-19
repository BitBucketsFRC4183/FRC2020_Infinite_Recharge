package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.Noise;
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
public class UnscentedKalmanFilter extends GenericKalmanFilter<StateSpaceModel, OutputObserver> {
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



    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, double alpha, double kappa, double beta) {
        super(sys);



        int L = sys.getModel().getNumStates();

        this.lambda = alpha * alpha * (L + kappa) - L;
        Wm0 = lambda / (lambda + L);
        Wc0 = lambda / (lambda + L) + (1 - alpha * alpha + beta * beta);
        W = 1.0 / (2 * (L + lambda));

        sigmaPoints = new SimpleMatrix[2*L + 1];
    }

    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, double alpha, double beta) {
        this(sys, alpha, 0, beta);
    }

    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, double alpha) {
        this(sys, alpha, 0, 2);
    }

    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys) {
        this(sys, Math.pow(10, -3), 0, 2);
    }



    @Override
    protected void predict() {
        calculateSigmaPoints();

        super.predict();
    }



    private void calculateSigmaPoints() {
        StateSpaceModel model = SYS.getModel();

        sigmaPoints[0] = model.getState();
        sigmaPoints[0] = model.propogate(sigmaPoints[0]);

        int L = model.getNumStates();

        SimpleMatrix xs = MathUtils.chol(P, true).scale(Math.sqrt(L + lambda));
        SimpleMatrix vec;

        x_apriori = sigmaPoints[0].scale(Wm0);

        for (int i = 1; i <= L; i++) {
            vec = xs.extractVector(true, i);

            sigmaPoints[i] = sigmaPoints[0].plus(vec);
            sigmaPoints[i] = model.propogate(sigmaPoints[i]);

            sigmaPoints[L + i] = sigmaPoints[0].minus(vec);
            sigmaPoints[L + i] = model.propogate(sigmaPoints[L + i]);
        }
    }



    @Override
    protected SimpleMatrix predictState() {
        StateSpaceModel model = SYS.getModel();
        int L = model.getNumStates();



        SimpleMatrix x = new SimpleMatrix(L, 1);

        x = sigmaPoints[0].scale(Wm0);

        for (int i = 1; i <= 2*L; i++) {
            x = x.plus(sigmaPoints[i].scale(W));
        }

        return x;
    }

    @Override
    protected SimpleMatrix predictP() {
        StateSpaceModel model = SYS.getModel();
        int L = model.getNumStates();


        
        eX = new SimpleMatrix[2*L + 1];
        SimpleMatrix p = new SimpleMatrix(L, L);

        for (int i = 0; i <= 2*L; i++) {
            eX[i] = (sigmaPoints[i].minus(x_apriori));

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

        return p.plus(processNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getS() {
        StateSpaceModel model = SYS.getModel();
        OutputObserver outputObserver = SYS.getObserver();
        int L = model.getNumStates();

        // outputs of transformed sigma points
        SimpleMatrix[] ys = new SimpleMatrix[2*L + 1];

        // already calculated transformed sigma points, just get the output
        for (int i = 0; i <= 2*L; i++) {
            ys[i] = outputObserver.getOutput(sigmaPoints[0], model.getInput(), model.getLastTime(), model.getCount());
        }

        SimpleMatrix yAvg = ys[0].scale(Wm0);
        for (int i = 1; i <= 2*L; i++) {
            yAvg = yAvg.plus(ys[i].scale(W));
        }

        eY = new SimpleMatrix[2*L + 1];
        for (int i = 0; i <= 2*L; i++) {
            eY[i] = ys[i].minus(yAvg);
        }

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

        return s.plus(outputNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getCxy() {
        StateSpaceModel model = SYS.getModel();
        OutputObserver outputObserver = SYS.getObserver();
        int L = model.getNumStates();

        // X by Y
        SimpleMatrix C = new SimpleMatrix(model.getNumStates(), outputObserver.getNumOutputs());
        SimpleMatrix cross;

        for (int i = 0; i <= 2*L; i++) {
            cross = eX[i].mult(eY[i].transpose());

            if (i == 0) {
                C = C.plus(cross.scale(Wc0));
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
}