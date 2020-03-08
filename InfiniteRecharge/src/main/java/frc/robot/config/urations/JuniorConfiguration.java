
package frc.robot.config.urations;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.Config;
import frc.robot.config.MotorConfig.EncoderType;
import frc.robot.utils.control.pidf.PIDF;

public class JuniorConfiguration extends Config {

    // either for driving or shooting
    private boolean forDriving = true;

    // Drive
    public int LEFT_DRIVE_IDS[] =
        (forDriving) ? (new int[]{1, 4})
        :              (new int[]{5, 8});
        
        
    public int RIGHT_DRIVE_IDS[] =
        (forDriving) ? (new int[]{2, 3})
        :              (new int[]{6, 7});

    // Shooter
    public int AZIMUTH_MOTOR_ID = 3;
    public int SHOOTER_MOTOR_ID = (forDriving) ? 6 : 13;
    public int FEEDER_MOTOR_ID = (forDriving) ? 7 : 2;
    
    // Intake
    public int INTAKE_MOTOR_ID = 8;

    
    public JuniorConfiguration() {
        super();

        name = "JUNIOR";

        // Subsystem Enablers
        enableShooterSubsystem = false;
        enableBallManagementSubsystem = false;
        enableDriveSubsystem = true;
        enableClimbSubsystem = false;
        enableIntakeSubsystem = false;
        enableSpinnyboiSubsystem = false;

        // junior doesn't have these, but move the motor ids around so they
        // don't conflict with junior's drive motors
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;

        shooter.azimuth.positionPIDF = new PIDF(
            0.1 * 1023f / 176 * 2 * 2 * 2 * 2,
            0,
            10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2,
            1023f / 2650
        );

        shooter.azimuth.motionMagicAcceleration = 1350;
        shooter.azimuth.motionMagicCruiseVelocity = 1350;
        shooter.azimuth.inverted = true;

        // setup junior's drive motors and pid constants.
        drive.MOTORS_PER_SIDE = 2;
        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        drive.gearRatio = 1;
        drive.ticksPerRevolution = 8192;
        drive.encoderType = EncoderType.Quadrature;
        drive.wheelRadius_in = 2;
        drive.trackWidth_in = 18.72;
        drive.characterization = new SimpleMotorFeedforward(1.23, 0.536, 0.204);
        drive.invertLeftCommand = false;
        drive.invertRightCommand = false;

        drive.maxAllowedSpeed_ips = 5 * 12.0;
        drive.maxAllowedTurn_degps = 180;

        drive.initMotorConfigArrays();

        drive.useFancyOdometry = false;

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