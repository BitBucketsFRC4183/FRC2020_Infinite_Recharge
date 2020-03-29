package frc.robot.subsystem.navigation.systems;

import org.ejml.simple.SimpleMatrix;

import frc.robot.config.Config;
import frc.robot.subsystem.navigation.NavigationConstants;
import frc.robot.utils.control.statespace.models.CStateSpaceModel;
import frc.robot.utils.control.statespace.observer.OutputObserver;
import frc.robot.utils.control.statespace.system.StateSpaceSystem;

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
public class RobotSystem1 extends StateSpaceSystem<CStateSpaceModel, OutputObserver> {
    // states
    private static final int X_LENGTH = 6;
    private static final int X_X = 1; // x position of robot (m)
    private static final int X_Y = 2; // y position of robot (m)
    private static final int X_THETA = 3; // heading of robot (rad)
    private static final int X_VL = 4; // velocity of left wheels (m/s)
    private static final int X_VR = 5; // velocity of right wheels (m/s)
    private static final int X_OFFSET = 6; // offset of yaw from NavX. 

    // outputs
    private static final int Y_LENGTH = 5;
    private static final int Y_TY = 1;
    private static final int Y_TX = 2;
    private static final int Y_VL = 3;
    private static final int Y_VR = 4;
    private static final int Y_YAW = 5; // yaw = theta + offset, where offset describes the initial heading of the robot



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


        return new CStateSpaceModel(
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
    }

    public static OutputObserver generateObserver(Config config, RobotSystemConfig systemConfig) {
        return new OutputObserver(6) {
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
    }



    public RobotSystem1(Config config, RobotSystemConfig systemConfig) {
        super(
            generateModel(config, systemConfig),
            generateObserver(config, systemConfig)
        );
    }
}