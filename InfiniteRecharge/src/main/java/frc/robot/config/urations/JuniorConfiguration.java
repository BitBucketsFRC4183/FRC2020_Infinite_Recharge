
package frc.robot.config.urations;

import frc.robot.config.Config;
import frc.robot.config.MotorConfig;
import frc.robot.utils.control.pidf.PIDF;

public class JuniorConfiguration extends Config {

    public int AZIMUTH_MOTOR_ID = 5;
    public int SHOOTER_MOTOR_ID = 6;
    public int INTAKE_MOTOR_ID = 7;

    public JuniorConfiguration() {
        super();

        // junior doesn't have these, but move the motor ids around so they
        // don't conflict with junior's drive motors
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.intake.id = INTAKE_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;

        // setup junior's drive motors and pid constants.
        drive.MOTORS_PER_SIDE = 2;
        drive.leftIDs = new int[]{1, 2};
        drive.rightIDs = new int[]{ 3, 4 };

        drive.initMotorConfigArrays();

        double velocityKp 	 = 0.14014*4;

        // Drive
        drive.leftLeader.velocityPIDF = new PIDF(
                0.14014*4, // P
                0.005, // I
                10 * velocityKp,	// D - Start with 10 x Kp for increased damping of overshoot
                0.05115, // F
                400 // iZone
        );
        drive.rightLeader.velocityPIDF = drive.leftLeader.velocityPIDF;

    }
}