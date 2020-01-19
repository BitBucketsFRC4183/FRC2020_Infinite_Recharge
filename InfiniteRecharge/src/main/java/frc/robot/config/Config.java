package frc.robot.config;

import frc.robot.utils.control.pidf.PIDF;

public class Config {

    //////////////////////////////////////////////////////////////////////////////
    // Motor IDs

    // Shooter
    public int AZIMUTH_MOTOR_ID = 1;
    public int SHOOTER_MOTOR_ID = 13;
    public int FEEDER_MOTOR_ID = 8;
    public int INTAKE_MOTOR_ID = 14;

    // Drive
    public int LEFT_DRIVE_LEADER_ID = 15;
    public int LEFT_DRIVE_FOLLOWER_ID = 2;
    public int RIGHT_DRIVE_LEADER_ID = 3;
    public int RIGHT_DRIVE_FOLLOWER_ID = 4;

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Motor Configs
    public static class ShooterConfig {
        public float gearRatio = 28f / 130f;
        public float defaultTurnVelocityDeg = 10;

        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig feeder = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();
    }
    public static class IntakeConfig {
       
        public MotorConfig intake = new MotorConfig();
    }

    public static class DriveConfig {
        public int MOTORS_PER_SIDE = 2;
        public MotorConfig leftMotors[];
        public MotorConfig rightMotors[];

        public int[] leftIDs;
        public int[] rightIDs;

        public MotorConfig leftLeader;
        public MotorConfig rightLeader;

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

                rightMotors[i] = new MotorConfig();
                rightMotors[i].id = rightIDs[i];

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
    public DriveConfig drive = new DriveConfig();
    public IntakeConfig intake = new IntakeConfig();

    public Config() {

        //////////////////////////////////////////////////////////////////////////////
        // IDs (Again)
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;
        intake.intake.id = INTAKE_MOTOR_ID;

        drive.leftIDs = new int[] { LEFT_DRIVE_LEADER_ID, LEFT_DRIVE_FOLLOWER_ID };
        drive.rightIDs = new int[] { RIGHT_DRIVE_LEADER_ID, RIGHT_DRIVE_FOLLOWER_ID };

        //////////////////////////////////////////////////////////////////////////////
        // PIDFs

        // Shooter
        shooter.azimuth.positionPIDF = new PIDF(//
                0.05, // P
                0, // I
                0, // D
                0 /// F
        );
        shooter.shooter.velocityPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );
        shooter.feeder.velocityPIDF = new PIDF(//
                0.1, // P
                0, // I
                0, // D
                0 /// F
        );

        // Drive
        drive.initMotorConfigArrays();

        drive.leftLeader.velocityPIDF = new PIDF(//
                0.1, // P
                0.1, // I
                0.1, // D
                0.1 /// F
        );
        drive.leftLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );
        drive.rightLeader.velocityPIDF = new PIDF(//
                0.1, // P
                0.1, // I
                0.1, // D
                0.1 /// F
        );
        drive.rightLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );

        //////////////////////////////////////////////////////////////////////////////
        // Ticks Per Revolution
        shooter.azimuth.ticksPerRevolution = 8192;
    }

}