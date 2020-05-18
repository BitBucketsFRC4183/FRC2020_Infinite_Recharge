package frc.robot.subsystem.navigation.systems;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.config.Config;
import frc.robot.subsystem.navigation.NavigationConstants;
import frc.robot.utils.control.statespace.models.C2D;
import frc.robot.utils.control.statespace.models.CStateSpaceModel;
import frc.robot.utils.control.statespace.models.StateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;
import frc.robot.utils.data.filters.statespace.kalman.UnscentedKalmanFilter;
import frc.robot.utils.data.noise.ConstantNoiseSource;
import frc.robot.utils.data.noise.Noise;
import frc.robot.utils.data.noise.NoiseSource;
import frc.robot.utils.math.MathUtils;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
/**
 * Primary robot system, considers all sensors.
 * In the future, we'll want to switch to a different system if a sensor
 * is down, such as the LimeLight not seeing a target. This setup would
 * also help if we're being pushed as then we can rely on the NavX IMU
 * and the LL to determine our position, without considering wheel encoders
 */
public class RobotSystem1 extends StateSpaceSystem<StateSpaceModel, OutputObserver> {
    // states
    private static final int X_LENGTH = 6;
    private static final int X_X = 0; // x position of robot (m)
    private static final int X_Y = 1; // y position of robot (m)
    private static final int X_THETA = 2; // heading of robot (rad)
    private static final int X_VL = 3; // velocity of left wheels (m/s)
    private static final int X_VR = 4; // velocity of right wheels (m/s)
    private static final int X_OFFSET = 5; // offset of yaw from NavX. 

    // outputs
    private static final int Y_LENGTH = 5;
    private static final int Y_TY = 0;
    private static final int Y_TX = 1;
    private static final int Y_VL = 2;
    private static final int Y_VR = 3;
    private static final int Y_YAW = 4; // yaw = theta + offset, where offset describes the initial heading of the robot



    private static SimpleMatrix tempVs = new SimpleMatrix(2, 1);
    private static SimpleMatrix tempUs = new SimpleMatrix(2, 1);

    public static CStateSpaceModel generateModel(Config config, RobotSystemConfig systemConfig) {
        // these error tolerances are also pretty arbitrary. the limiting factor in the KF here
        // is not error tolerances in our differential equation solver, but rather system and
        // sensor noise
        double ex = 0.01/5 * NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S; // 1cm/5 /s * NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S s/iteration
        double etheta = Math.PI / 180 * NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S; // 1deg / 5s
        double ev = 0.001 * NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S; // 1mm/s

        double[] errTol = new double[X_LENGTH];
        errTol[X_X] = ex;
        errTol[X_Y] = ex;
        errTol[X_THETA] = etheta;
        errTol[X_VL] = ev;
        errTol[X_VR] = ev;
        errTol[X_OFFSET] = etheta;

        CStateSpaceModel model =  new CStateSpaceModel(
            X_LENGTH,
            2, // volts to left and right motors
            NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S,
            0.001, // 1ms
            NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S / 2, // doesn't really matter as long as the error's acceptable
            errTol,
            errTol
        ) {
            @Override
            public void deriv(double[] x, double[] u, double t, int k, double[] xDot) {
                double theta = x[X_THETA];
                double vL = x[X_VL];
                double vR = x[X_VR];
                double v = (vL + vR) / 2;

                tempVs.set(0, vL);
                tempVs.set(1, vR);



                double VL = systemConfig.leftVolts.getAsDouble();
                double VR = systemConfig.rightVolts.getAsDouble();

                tempUs.set(0, VL);
                tempUs.set(1, VR);
                
                SimpleMatrix as = config.drive.Ac_char.mult(tempVs).plus(config.drive.Bc_char.mult(tempUs));

                xDot[X_X] = v * Math.cos(theta);
                xDot[X_Y] = v * Math.sin(theta);
                xDot[X_THETA] = (vR - vL) / (config.drive.trackWidth_in * NavigationConstants.IN_TO_M);
                xDot[X_VL] = as.get(0);
                xDot[X_VR] = as.get(1);
                xDot[X_OFFSET] = 0; // never changes (theoretically). In reality, IMU drifts a bit so it's okay if it changes
            }
        };

        SimpleMatrix Qc = new SimpleMatrix(X_LENGTH, X_LENGTH);
        Qc.insertIntoThis(X_VL, X_VL, config.drive.QVc_char);

        model.addNoiseSource(new NoiseSource(X_LENGTH, Y_LENGTH) {
            @Override
            public Noise getNoise(SimpleMatrix x, SimpleMatrix u, double t, int k) {
                double theta = x.get(X_THETA);
                double vL = x.get(X_VL);
                double vR = x.get(X_VR);
                double v = (vL + vR) / 2;



                // gotta linearize sometimes :(
                SimpleMatrix A = new SimpleMatrix(X_LENGTH, X_LENGTH);

                A.set(X_X, X_VL, Math.cos(theta) / 2);
                A.set(X_X, X_VR, Math.cos(theta) / 2);
                A.set(X_X, X_THETA, -v * Math.sin(theta));

                A.set(X_Y, X_VL, Math.sin(theta) / 2);
                A.set(X_Y, X_VR, Math.sin(theta) / 2);
                A.set(X_Y, X_THETA, v * Math.cos(theta));

                A.set(X_THETA, X_VL, -1 / (config.drive.trackWidth_in * NavigationConstants.IN_TO_M));
                A.set(X_THETA, X_VR,  1 / (config.drive.trackWidth_in * NavigationConstants.IN_TO_M));

                A.insertIntoThis(X_VL, X_VL, config.drive.Ac_char);



                SimpleMatrix F = new SimpleMatrix(2*X_LENGTH, 2*X_LENGTH);
                F.insertIntoThis(0, 0, A.negative());
                F.insertIntoThis(X_LENGTH, X_LENGTH, A.transpose());
                F.insertIntoThis(0, X_LENGTH, Qc);

                // ??? why does SimpleMatrix have divide but not multiply lol
                SimpleMatrix G = MathUtils.expm(F.divide(1.0 / NavigationConstants.KALMAN_FILTER_UPDATE_RATE_S));

                SimpleMatrix S = C2D.getS(A, 0.02);
                SimpleMatrix Ad = C2D.getA(S, X_LENGTH);

                // get an approximation for the noise covariance
                SimpleMatrix Q = Ad.mult(G.extractMatrix(0, X_LENGTH, X_LENGTH, 2*X_LENGTH));

                return new Noise(Q);
            }
        });

        return model;
    }

    public static OutputObserver generateObserver(Config config, RobotSystemConfig systemConfig) {
        OutputObserver observer = new OutputObserver(6) {
            @Override
            public SimpleMatrix getOutput(SimpleMatrix state, SimpleMatrix input, double t, int k) {
                double theta = state.get(X_THETA);
                double azimuth = Math.toRadians(systemConfig.azimuthDegrees.getAsDouble());

                double x = state.get(X_X)
                    + config.shooter.xTurretCenter * Math.cos(theta)
                    - config.shooter.yTurretCenter * Math.sin(theta) // center of turret doesn't rotate with turret, only robot
                    + config.shooter.rLL * Math.cos(theta + azimuth);
                
                double y = state.get(X_Y)
                    + config.shooter.xTurretCenter * Math.sin(theta)
                    + config.shooter.yTurretCenter * Math.cos(theta) // center of turret doesn't rotate with turret, only robot
                    + config.shooter.rLL * Math.sin(theta + azimuth);
                
                // calcualte distance to target and convert to inches
                double d = Math.sqrt(
                    Math.pow(x - NavigationConstants.TARGET_X, 2)
                    + Math.pow(y - NavigationConstants.TARGET_Y, 2)
                ) / NavigationConstants.IN_TO_M;

                double tx = Math.atan2(NavigationConstants.TARGET_Y - y, NavigationConstants.TARGET_X - x);



                double[] out = new double[Y_LENGTH];

                out[Y_TX] = tx;
                out[Y_TY] = systemConfig.tyFromDistance.apply(d);
                out[Y_VL] = state.get(X_VL);
                out[Y_VR] = state.get(X_VR);
                out[Y_YAW] = theta + state.get(X_OFFSET);

                return new SimpleMatrix(Y_LENGTH, 1, true, out);
            }
        };

        SimpleMatrix R = new SimpleMatrix(Y_LENGTH, Y_LENGTH);
        R.set(Y_TY, Y_TY, systemConfig.ll_angle_var);
        R.set(Y_TX, Y_TX, systemConfig.ll_angle_var);
        R.set(Y_YAW, Y_YAW, systemConfig.imu_yaw_var);

        observer.addNoiseSource(new ConstantNoiseSource(Y_LENGTH, R));

        return observer;
    }



    private Config config;
    private RobotSystemConfig systemConfig;

    public RobotSystem1(Config config, RobotSystemConfig systemConfig) {
        super(
            generateModel(config, systemConfig),
            generateObserver(config, systemConfig)
        );

        this.config = config;
        this.systemConfig = systemConfig;

        setFilter(createFilter());
    }



    private UnscentedKalmanFilter createFilter() {
        SimpleMatrix P0 = new SimpleMatrix(X_LENGTH, X_LENGTH);
        P0.set(X_X, X_X, systemConfig.x0_var);
        P0.set(X_Y, X_Y, systemConfig.y0_var);
        P0.set(X_THETA, X_THETA, systemConfig.theta_var);
        P0.set(X_THETA, X_OFFSET, systemConfig.theta_var);
        P0.set(X_OFFSET, X_THETA, systemConfig.theta_var);
        P0.set(X_OFFSET, X_OFFSET, systemConfig.theta_var);
        P0.set(X_VL, X_VL, systemConfig.velocity_var);
        P0.set(X_VR, X_VR, systemConfig.velocity_var);

        // we love Java
        return new UnscentedKalmanFilter(this, P0);
    }

    public void setInitialPose(Pose2d pose) {
        SimpleMatrix x0 = new SimpleMatrix(X_LENGTH, 1);

        Translation2d translation = pose.getTranslation();
        // inverse of FieldConstants.transformToRobot
        x0.set(X_X, 0, translation.getY());
        x0.set(X_Y, 0, -translation.getX());
        x0.set(X_THETA, pose.getRotation().getRadians());
    }
}