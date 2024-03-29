package frc.robot.config;

import frc.robot.config.MotorConfig;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterConfig implements Loggable {

    // Constants

    public static final boolean USE_AZIMUTH_FILTER = false;
    public static final int FILTER_LENGTH = 25;
    public static final int FEEDER_FILTER_LENGTH = 4;

    public static final double SHOOTER_FLYWHEEL_RADIUS = 2; // inches

    public static final double FEEDER_OUTPUT_PERCENT = 0.8;

    public static final boolean USE_BANG_BANG = false;
    public static final double BANG_BANG_PERCENT = 1.0;
    public static final double BANG_BANG_RAMP_UP_PERCENT = 0.4;
    public static final int BANG_BANG_ERROR = 0;
    public static final double BANG_BANG_MAINTAIN_SPEED_PERCENT = 0.6;
    public static final int DEFAULT_ELEVATION_TARGET_DEG = 55;
    public static final float ELEVATION_LIMIT_SWITCH_DEG = 0f;
    public static final float AZIMUTH_LEFT_LIMIT_SWITCH_DEG = 90;
    public static final float AZIMUTH_RIGHT_LIMIT_SWITCH_DEG = 180;

    // Dashboard Config

    private float shooterVelocity_rpm;
    private float azimuthTurnRate_deg;
    private float elevationTurnRate_deg;

    // Config

    public float azimuthGearRatio = 28f / 130f;

    public float elevationGearRatio = 40f / 70f;

    public float shooterGearRatio = .45f / 1f;

    public double manualAzimuthDeadband = 0.2;
    public double manualElevationDeadband = 0.2;

    public float leftAzimuthSoftLimit_deg = 90;
    public float rightAzimuthSoftLimit_deg = 180;

    public float forwardElevationSoftLimit_deg = 60;
    public float backwardElevationSoftLimit_deg = 0;

    public float feederSpinUpDeadband_ticks = 100;

    public MotorConfig azimuth = new MotorConfig();
    public MotorConfig elevation = new MotorConfig();
    public MotorConfig feeder = new MotorConfig();
    public MotorConfig shooter = new MotorConfig();
    public MotorConfig shooterFollower = new MotorConfig();

    // meters as all good things are
    // relative to center of robot
    // not necessarily CoM of robot but like, same thing
    public double xTurretCenter = 0.17181322;
    public double yTurretCenter = 0.01766824;
    // distance from center of turret to LL camera
    public double rLL = 0.19368008;

    public float[] elevationPositions_deg = new float[] { //
            0, /// Comment so that VSCode doesn't ruin my format.
            10, // Comment so that VSCode doesn't ruin my format.
            40, // Comment so that VSCode doesn't ruin my format.
            50, // Comment so that VSCode doesn't ruin my format.
            60 /// Comment so that VSCode doesn't ruin my format.
    };

    public ShooterConfig() {
        shooter.encoderType = MotorConfig.EncoderType.Integrated;
        shooter.inverted = true;
        shooterFollower.inverted = true;

        azimuth.motionMagicAcceleration = 1350;
        azimuth.motionMagicCruiseVelocity = 1350;

        elevation.motionMagicAcceleration = 1350;
        elevation.motionMagicCruiseVelocity = 1350;

        azimuth.inverted = true;
        elevation.inverted = false;
    }

    @Config(defaultValueNumeric = 10, name = "Azimuth Turn Rate (Degrees)")
    public void setAzimuthTurnRate_deg(float azimuthTurnRate_deg) {
        this.azimuthTurnRate_deg = azimuthTurnRate_deg;
    }

    @Config(defaultValueNumeric = 10, name = "Elevation Turn Rate (Degrees)")
    public void setElevationTurnRate_deg(float elevationTurnRate_deg) {
        this.elevationTurnRate_deg = elevationTurnRate_deg;
    }

    @Config(defaultValueNumeric = 6000, name = "Shooter Velocity (RPM)")
    public void setShooterVelocity_rpm(float shooterVelocity_rpm) {
        this.shooterVelocity_rpm = shooterVelocity_rpm;
    }

    @Override
    public String configureLogName() {
        return "Shooter";
    }

    public float getAzimuthTurnRate_deg() {
        return azimuthTurnRate_deg;
    }

    public float getElevationTurnRate_deg() {
        return elevationTurnRate_deg;
    }

    public float getShooterVelocity_rpm() {
        return shooterVelocity_rpm;
    }
}