package frc.robot.config;

import frc.robot.utils.control.pidf.PIDF;

public class Config {


    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Shooter
    public int AZIMUTH_MOTOR_ID = 1;
    public int SHOOTER_MOTOR_ID = 13;
    public int INTAKE_MOTOR_ID = 3;

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Stuff that makes this work

    public static class ShooterConfig {
        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig intake = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();
    }

    public ShooterConfig shooter = new ShooterConfig();

    public Config() {

    //////////////////////////////////////////////////////////////////////////////
    // IDs (Again)

        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.intake.id = INTAKE_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;

    //////////////////////////////////////////////////////////////////////////////
    // PIDFs
        shooter.azimuth.positionPIDF = new PIDF(
        1, // P
        1, // I
        1, // D
        1 /// F
        );
        shooter.shooter.velocityPIDF = new PIDF(
        1, // P
        1, // I
        1, // D
        1 /// F
        );
    }

    
}