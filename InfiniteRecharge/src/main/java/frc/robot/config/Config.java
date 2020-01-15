package frc.robot.config;

import frc.robot.utils.control.pidf.PIDF;

public class Config {

    //////////////////////////////////////////////////////////////////////////////
    // Motor IDs

    // Shooter
    public int AZIMUTH_MOTOR_ID = 1;
    public int SHOOTER_MOTOR_ID = 13;
    public int INTAKE_MOTOR_ID = 8;

    // Drive

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Motor Configs
    public static class ShooterConfig {
        public float gearRatio = 28f / 130f;
        public int azimuthMotorTicks = 8192;
        public float defaultTurnVelocityDeg = 10;

        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig intake = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();
    }

    public static class DriveConfig {
        public int MOTORS_PER_SIDE = 2;
        public MotorConfig leftMotors[];
        public MotorConfig rightMotors[];

        public int[] leftIDs = { 15, 2 };
        public int[] rightIDs = { 3, 4 };

        public MotorConfig leftLeader;
        public MotorConfig rightLeader;

        public DriveConfig() {
            initMotorConfigArrays();
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

    public Config() {

        //////////////////////////////////////////////////////////////////////////////
        // IDs (Again)
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.intake.id = INTAKE_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;

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

        // Drive
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
    }

}