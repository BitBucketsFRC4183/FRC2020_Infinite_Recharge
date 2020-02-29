package frc.robot.subsystem.scoring.shooter;

public class ShooterConstants {

    public static final boolean USE_AZIMUTH_FILTER = false;
    public static final int FILTER_LENGTH = 25;
    public static final int FEEDER_FILTER_LENGTH = 4;

    public static final double SHOOTER_FLYWHEEL_RADIUS = 2; // inches

    public static final double FEEDER_OUTPUT_PERCENT = 0.8;

    public static final float DEFAULT_SHOOTER_VELOCITY_RPM = 4183;

    public static final boolean USE_BANG_BANG = true;
    public static final double BANG_BANG_PERCENT = 1.0;
    public static final double BANG_BANG_RAMP_UP_PERCENT = 0.5;
    public static final int BANG_BANG_ERROR = 0;
    public static final double BANG_BANG_MAINTAIN_SPEED_PERCENT = 0.6;
}