
package frc.robot.config.urations;

import frc.robot.config.Config;
import frc.robot.utils.control.pidf.PIDF;

public class JuniorConfiguration extends Config {

    // Drive
    public int LEFT_DRIVE_IDS[] = {1, 2};
    public int RIGHT_DRIVE_IDS[] = {3, 4};

    // Shooter
    public int AZIMUTH_MOTOR_ID = 5;
    public int SHOOTER_MOTOR_ID = 6;
    public int FEEDER_MOTOR_ID = 7;
    
    // Intake
    public int INTAKE_MOTOR_ID = 8;

    
    public JuniorConfiguration() {
        super();

        // junior doesn't have these, but move the motor ids around so they
        // don't conflict with junior's drive motors
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;

        shooter.azimuth.positionPIDF = new PIDF(
            0.1 * 1023 / 176 * 2 * 2 * 2 * 2,
            0,
            10 * 0.1 * 1023 / 176 * 2 * 2 * 2 * 2,
            1023 / 2650
        );

        // setup junior's drive motors and pid constants.
        drive.MOTORS_PER_SIDE = 2;
        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        drive.initMotorConfigArrays();

        double velocityKp = 0.14014 * 4;

        // Drive
        drive.leftLeader.velocityPIDF = new PIDF(
                0.14014/4,//velocityKp, // P
                0,//0.005, // I
                10 * 0.14014/4,//10 * velocityKp, // D - Start with 10 x Kp for increased damping of overshoot
                1023.0 / 17300, // F
                400 // iZone
        );
        drive.rightLeader.velocityPIDF = drive.leftLeader.velocityPIDF;

    }
}