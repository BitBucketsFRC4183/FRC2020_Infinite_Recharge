package frc.robot.utils.data.filters.statespace.kalman;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.noise.Noise;
import frc.robot.utils.data.statistics.transforms.UnscentedTransform;
import frc.robot.utils.data.statistics.transforms.UnscentedTransform.Distribution;
import frc.robot.utils.data.statistics.transforms.UnscentedTransform.SigmaPointsSettings;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
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
    private final SigmaPointsSettings sigmaPointSettings;
    private final UnscentedTransform updateX;
    private final UnscentedTransform yOutput;

    private Distribution distX;
    private Distribution distY;




    /**
     * Create an Unscented Kalman Filter for a system
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param sigmaPointSettings settings used to determine spread of sigma points used in unscented transform
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, SigmaPointsSettings sigmaPointSettings) {
        super(sys);



        // initialize the estimate covariance
        P = P0;

        this.sigmaPointSettings = sigmaPointSettings;
        updateX = new UnscentedTransform() {
            @Override
            public SimpleMatrix f(SimpleMatrix x) {
                return SYS.getModel().propogate(x);
            }
        };

        yOutput = new UnscentedTransform() {
            @Override
            public SimpleMatrix f(SimpleMatrix x) {
                return SYS.getObserver().getOutput(
                    x,
                    SYS.getModel().getInput(),
                    SYS.getModel().getLastTime(),
                    SYS.getModel().getCount()
                );
            }
        };
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa is defaulted to 0
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha see documentation for {@link UnscentedTransform}
     * @param kappa see documentation for {@link UnscentedTransform}
     * @param beta see documentation for {@link UnscentedTransform}
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha, double kappa, double beta) {
        this(sys, P0, new SigmaPointsSettings(sys.getModel().getNumStates(), alpha, beta, kappa));
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa is defaulted to 0
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha see documentation for {@link UnscentedTransform}
     * @param beta see documentation for {@link UnscentedTransform}
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha, double beta) {
        this(sys, P0, new SigmaPointsSettings(sys.getModel().getNumStates(), alpha, beta));
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa is defaulted to 0
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha see documentation for {@link UnscentedTransform}
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0, double alpha) {
        this(sys, P0, new SigmaPointsSettings(sys.getModel().getNumStates(), alpha));
    }

    /**
     * Create an Unscented Kalman Filter for a system. kappa and beta are defaulted to 0 and 2 respectively
     * 
     * @param sys any state space system to be filtered
     * @param P0 initial state estimate covariance
     * @param alpha tuning parameter that controls spread of sigma points, usually made
     *     really small such as 10^-3
     */
    public UnscentedKalmanFilter(StateSpaceSystem<StateSpaceModel, OutputObserver> sys, SimpleMatrix P0) {
        this(sys, P0, new SigmaPointsSettings(sys.getModel().getNumStates()));
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

        SimpleMatrix x = model.getState();

        // generate the approximate distribution from sigma points around x with covariance P
        distX = updateX.approximateDistribution(sigmaPointSettings.generateSigmaPoints(x, P));
    }



    @Override
    protected SimpleMatrix predictState() {
        // return mean of distribution
        return distX.getMean();
    }

    @Override
    protected SimpleMatrix predictP() {
        StateSpaceModel model = SYS.getModel();

        Noise processNoise = model.getNoiseSource().getNoise(
            model.getState(),
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        // add in the process noise's covariance because real world
        // systems b like that
        return distX.getCovariance().plus(processNoise.getCovariance());
    }

    @Override
    protected SimpleMatrix getS() {
        StateSpaceModel model = SYS.getModel();
        OutputObserver outputObserver = SYS.getObserver();

        // a very hacky way to deal with it!
        distY = yOutput.approximateDistribution(distX.getSigmaPoints(), sigmaPointSettings);

        Noise outputNoise = outputObserver.getNoiseSource().getNoise(
            x_apriori,
            model.getInput(),
            model.getLastTime(),
            model.getCount()
        );

        // there's always some noise from the output so just add
        // its covariance
        return distY.getCovariance().plus(outputNoise.getCovariance());
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

        SimpleMatrix[] eX = distX.getDeviations();
        SimpleMatrix[] eY = distY.getDeviations();

        for (int i = 0; i <= 2*L; i++) {
            // calculate this sigma point's individual cross variance
            cross = eX[i].mult(eY[i].transpose());

            // add it to the bunch
            if (i == 0) {
                C = cross.scale(sigmaPointSettings.getWc0());
            } else {
                C = C.plus(cross.scale(sigmaPointSettings.getW()));
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
        return distY.getMean();
    }
}