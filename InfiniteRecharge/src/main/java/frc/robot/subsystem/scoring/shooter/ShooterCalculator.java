package frc.robot.subsystem.scoring.shooter;

import java.util.Arrays;
import java.util.List;

import frc.robot.subsystem.vision.VisionSubsystem;

import frc.robot.utils.math.MathUtils;
import frc.robot.config.Config;

public class ShooterCalculator {

    // Arraylist of velocity points, used to calculate the spline
    private final List<VelocityPoint> points;
    private boolean targetLocked = false;
    VisionSubsystem visionSubsystem;
    SplineVelocityPoint splineVPoint;
    private double lastTargetHoodAngle;
    private double lastTargetVelocity;

    private final Config config = new Config();

    public static class VelocityPoint implements Comparable<VelocityPoint> {
        final double distance_in;
        final double speed_rpm;
        final double hoodAngle_deg;

        public VelocityPoint(final double distance_in, final double speed_rpm, final double hoodAngle) {
            this.distance_in = distance_in;
            this.speed_rpm = speed_rpm;
            this.hoodAngle_deg = hoodAngle;
        }

        public VelocityPoint(final double distance_in) {
            this.distance_in = distance_in;
            this.speed_rpm = 0;
            this.hoodAngle_deg = 0;
        }

        @Override
        public int compareTo(VelocityPoint other) {
            return Double.compare(this.distance_in, other.distance_in);
        }

		public double getDistance_in() {
			return distance_in;
		}

		public double getSpeed_rpm() {
			return speed_rpm;
		}

		public double getHoodAngle_deg() {
			return hoodAngle_deg;
		}
    }

    /**
     * Only unit tests should call this constructor
     * @param points
     */
    ShooterCalculator(List<VelocityPoint> points) {
        this.points = points;
    }

    /**
     * Default constructor with our calculated data points
     * These points were obtained by testing the robot irl at multiple points
     */
    public ShooterCalculator() {
        this(Arrays.asList(
            new VelocityPoint(109, 4500, 43), // initiation line
            new VelocityPoint(130, 4500, 43.5),
            new VelocityPoint(150, 4500, 43),
            new VelocityPoint(196, 5200, 46),
            new VelocityPoint(206, 5050, 45),
            new VelocityPoint(323, 6100, 53),
            new VelocityPoint(397, 7400, 59)
            )
        );
    }

    public void initialize(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.splineVPoint = new SplineVelocityPoint(points);
    }

    /**
     * If we have a valid target, then get the distance and then return the needed RPM to make the shot
     * TODO: test empirically
     * @return needed RPM to make the shot
     */
    public double calculateSpeed_rpm() {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);

            targetLocked = true;
            return splineVPoint.getSpeedSpline(distance);
        }
        // calculate the velocity for this distance_in
        // If we don't have a target locked (we've lost the target) then return a default velocity
        if (!targetLocked){
            return ShooterConstants.DEFAULT_SHOOTER_VELOCITY_RPM;
        } else {
            return lastTargetVelocity;
        }
    }

    /**
     * Gets the speed in rpm and converts it to ticks
     * @return gets needed speed in ticks
     */
    public double calculateSpeed_ticks() {
        double speed_rpm = calculateSpeed_rpm();
        double speed_ticks = MathUtils.unitConverter(speed_rpm, 600, config.shooter.shooter.ticksPerRevolution) * config.shooter.shooterGearRatio;

        return speed_ticks;
    }

    // TODO: empirically test this
    // decreases as u get closer
    // increases as u get it farther
    /**
     * Gets the distance if we have a valid target then gets the needed angle from the spline
     * @return hood angle needed to make a shot
     */
    public double calculateHoodAngle_deg() {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);
            double hoodAngle = splineVPoint.getAngleSpline(distance);
            // Store the target angle in case we lose the target.
            if (!targetLocked){
                lastTargetHoodAngle = hoodAngle;
            }
            targetLocked = true;

            return hoodAngle;
        }
        // calculate the hood angle for this distance_in
        // If we don't have a target locked (we've lost the target) then return a default angle
        if (!targetLocked){
            return ShooterConstants.DEFAULT_ELEVATION_TARGET_DEG;
        } else {
            return lastTargetHoodAngle;
        }
    }
    public void setTargetLocked(boolean b){
        targetLocked = b;
    }
}
