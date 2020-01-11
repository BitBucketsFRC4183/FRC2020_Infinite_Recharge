package frc.robot.config;

public class Config {


    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Shooter
    public int AZIMUTH_MOTOR_ID = 1;
    public int SHOOTER_MOTOR_ID = 2;
    public int INTAKE_MOTOR_ID = 3;

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Categories & Stuff that makes this work

    public static class ShooterConfig {
        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig intake = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();
    }

    public ShooterConfig shooter = new ShooterConfig();

    public Config() {
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.azimuth.kp = 1f;
        shooter.azimuth.ki = 0.00001f;
        shooter.azimuth.kd = 10f;
        shooter.azimuth.kf = 1f;

        shooter.intake.id = INTAKE_MOTOR_ID;

        shooter.shooter.id = SHOOTER_MOTOR_ID;
        shooter.shooter.kp = 1f;
        shooter.shooter.ki = 0.00001f;
        shooter.shooter.kd = 10f;
        shooter.shooter.kf = 1f;
        
    }

    
}