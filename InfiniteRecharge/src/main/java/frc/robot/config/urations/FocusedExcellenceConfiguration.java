
package frc.robot.config.urations;

import frc.robot.config.Config;
import frc.robot.config.MotorConfig.EncoderType;
import frc.robot.utils.control.pidf.PIDF;

public class FocusedExcellenceConfiguration extends Config {
    public int AZIMUTH_MOTOR_ID = 0;
    public int ELEVATION_MOTOR_ID = 0;
    public int SHOOTER_MOTOR_ID = 0;
    public int FEEDER_MOTOR_ID = 0;

    public int INTAKE_MOTOR_ID = 0;

    public int LEFT_DRIVE_IDS[] = { 3, 2, 1 };
    public int RIGHT_DRIVE_IDS[] = { 6, 5, 4 };

    public int SPINNYBOI_MOTOR_ID = 0;



    public FocusedExcellenceConfiguration() {
        super();

        name = "Focused Excellence";

        drive.MOTORS_PER_SIDE = 3;
        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        drive.gearRatio = 1;
        drive.ticksPerRevolution = 8192;
        drive.encoderType = EncoderType.Quadrature;
        drive.wheelRadius_in = 3;
        drive.trackWidth_in = 23.5;

        drive.maxAllowedSpeed_ips = 8 * 12.0;
        drive.maxAllowedTurn_degps = 180;

        drive.invertLeftCommand = false;
        drive.invertRightCommand = false;

        drive.leftLeader.velocityPIDF = new PIDF(
            0.5115/2/1.5,
            0.0001,
            10.0*0.5115,
            0.113039,
            200
        );

        drive.rightLeader.velocityPIDF = new PIDF(
            0.683333/2/2,
            0.0001,
            10*0.683333/2/1.5,
            0.114944,
            200
        );

        drive.encoderType = EncoderType.Quadrature;

        drive.useFancyOdometry = false;

        drive.initMotorConfigArrays();
    }
}