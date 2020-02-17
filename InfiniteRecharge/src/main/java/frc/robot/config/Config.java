package frc.robot.config;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.Config.ShooterConfig.BallManagementConfig;
import frc.robot.utils.control.pidf.PIDF;

public class Config {

    //////////////////////////////////////////////////////////////////////////////
    // Subsystem Enablers

    public boolean enableShooterSubsystem = true;
    public boolean enableBallManagementSubsystem = true;
    public boolean enableDriveSubsystem = false;
    public boolean enableClimbSubsystem = true;
    public boolean enableIntakeSubsystem = true;
    public boolean enableSpinnyboiSubsystem = true;

    //////////////////////////////////////////////////////////////////////////////
    // Motor IDs

    // Shooter
    public int AZIMUTH_MOTOR_ID = 11;
    public int ELEVATION_MOTOR_ID = 14;
    public int SHOOTER_MOTOR_ID = 13;
    public int FEEDER_MOTOR_ID = 9;

    // Intake
    public int INTAKE_MOTOR_ID = 5;

    // Drive
    public int LEFT_DRIVE_IDS[] = { 2, 3 };
    public int RIGHT_DRIVE_IDS[] = { 1, 4 };

    // SpinnyBoi
    public int SPINNYBOI_MOTOR_ID = 6;
    public int BALLMANAGEMENT_MOTOR_ID = 10;

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Motor Configs
    public static class ShooterConfig {
        public float azimuthGearRatio = 28f / 130f;

        public float elevationGearRatio = 40f / 70f;

        public float shooterGearRatio = .48f / 1f;

        public float defaultAzimuthTurnVelocity_deg = 10;
        public float defaultElevationTurnVelocity_deg = 10;

        public double manualAzimuthDeadband = 0.2;
        public double manualElevationDeadband = 0.2;

        public float rightAzimuthSoftLimit_deg = 45;
        public float leftAzimuthSoftLimit_deg = 45;
        public float forwardElevationSoftLimit_deg = 60;
        public float backwardElevationSoftLimit_deg = 0;

        public float feederSpinUpDeadband_ticks = 600;

        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig elevation = new MotorConfig();
        public MotorConfig feeder = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();

        public float[] elevationPositions_deg = new float[] { //
                0, /// Comment so that VSCode doesn't ruin my format.
                10, // Comment so that VSCode doesn't ruin my format.
                25, // Comment so that VSCode doesn't ruin my format.
                45, // Comment so that VSCode doesn't ruin my format.
                60 /// Comment so that VSCode doesn't ruin my format.
        };

        public static class BallManagementConfig {
            public MotorConfig spinner = new MotorConfig();

            public BallManagementConfig() {
                spinner.encoderType = MotorConfig.EncoderType.None;
            }
        }

        public ShooterConfig() {
            shooter.encoderType = MotorConfig.EncoderType.Integrated;

            azimuth.motionMagicAcceleration = 1350;
            azimuth.motionMagicCruiseVelocity = 1350;

            elevation.motionMagicAcceleration = 1350;
            elevation.motionMagicCruiseVelocity = 1350;

            azimuth.inverted = true;
            elevation.inverted = false;
        }
    }

    public static class IntakeConfig {
        public boolean intakePivotEnabled = false;

        public MotorConfig intake = new MotorConfig();
    }

    public static class SpinnyBoiConfig {
        public MotorConfig spinner = new MotorConfig();
    }

    public static class DriveConfig {
        public double maxAllowedSpeed_ips = 8 * 12.0;
        public double maxAllowedTurn_degps = 180;

        public int MOTORS_PER_SIDE = 2;
        public MotorConfig leftMotors[];
        public MotorConfig rightMotors[];

        public int[] leftIDs;
        public int[] rightIDs;

        public MotorConfig leftLeader;
        public MotorConfig rightLeader;

        public boolean leftInverted = false;
        public boolean rightInverted = false;
        public boolean invertLeftCommand = false;
        public boolean invertRightCommand = true;

        public MotorConfig.EncoderType encoderType = MotorConfig.EncoderType.Integrated;

        /** Gear ratio from encoder to wheel. gearRatio encoder turns = 1 wheel turn */
        public double gearRatio = (10 + 8.0 / 9);
        public double ticksPerRevolution = 2048;
        public double wheelRadius_in = 3;
        public double trackWidth_in = 22.65;

        public double ROTATION_DRIVE_KP = 5 * 2 * Math.PI / 360;

        public SimpleMotorFeedforward characterization = new SimpleMotorFeedforward(1.23, 0.536, 0.204);;

        public DriveConfig() {
        }

        /**
         * Based on the MOTORS_PER_SIDE
         */
        public void initMotorConfigArrays() {
            leftMotors = new MotorConfig[MOTORS_PER_SIDE];
            rightMotors = new MotorConfig[MOTORS_PER_SIDE];
            for (int i = 0; i < MOTORS_PER_SIDE; i++) {
                leftMotors[i] = new MotorConfig();
                leftMotors[i].id = leftIDs[i];
                leftMotors[i].encoderType = encoderType;
                leftMotors[i].inverted = leftInverted;

                rightMotors[i] = new MotorConfig();
                rightMotors[i].id = rightIDs[i];
                rightMotors[i].encoderType = encoderType;
                rightMotors[i].inverted = rightInverted;

                if (i > 0) {
                    leftMotors[i].followingID = leftIDs[0];
                    rightMotors[i].followingID = rightIDs[0];
                }
            }
            leftLeader = leftMotors[0];
            rightLeader = rightMotors[0];
        }
    }

    public ShooterConfig shooter = new ShooterConfig();
    public BallManagementConfig ballManagement = new BallManagementConfig();
    public DriveConfig drive = new DriveConfig();
    public IntakeConfig intake = new IntakeConfig();
    public SpinnyBoiConfig spinnyboi = new SpinnyBoiConfig();

    public Config() {

        //////////////////////////////////////////////////////////////////////////////
        // IDs (Again)

        // Shooter
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.elevation.id = ELEVATION_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;
        ballManagement.spinner.id = BALLMANAGEMENT_MOTOR_ID;

        // Intake
        intake.intake.id = INTAKE_MOTOR_ID;

        // SpinnyBoi
        spinnyboi.spinner.id = SPINNYBOI_MOTOR_ID;

        // Drive
        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        //////////////////////////////////////////////////////////////////////////////
        // PIDFs

        // Shooter
        shooter.azimuth.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650 /// F
        );
        shooter.elevation.positionPIDF = new PIDF(//
                0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // P
                0, // I
                10 * 0.1 * 1023f / 176 * 2 * 2 * 2 * 2, // D
                1023f / 2650 /// F
        );
        shooter.shooter.velocityPIDF = new PIDF(//
                .1 * 1023. / 2300., // P
                0, // I
                0, // D
                1023. / 12000 /// F
        );

        // SpinnyBoi
        spinnyboi.spinner.positionPIDF = new PIDF(//
                0.05, // P
                0, // I
                0, // D
                0 /// F
        );

        // Drive
        drive.initMotorConfigArrays();

        drive.leftLeader.velocityPIDF = new PIDF(//
                0.1 * 1023 / 1000 / 4, // P
                0.0, // I
                0.1 * 1023 / 1000 / 4 * 10, // D
                1023 / 21740f /// F
        );
        drive.leftLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );
        drive.rightLeader.velocityPIDF = new PIDF(//
                0.1 * 1023 / 1000 / 4, // P
                0.0, // I
                0.1 * 1023 / 1000 / 4 * 10, // D
                1023 / 21340f /// F
        );
        drive.rightLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );

        //////////////////////////////////////////////////////////////////////////////
        // Ticks Per Revolution
        shooter.azimuth.ticksPerRevolution = 4096;
        shooter.elevation.ticksPerRevolution = 8192;
        shooter.shooter.ticksPerRevolution = 2048;
    }

}