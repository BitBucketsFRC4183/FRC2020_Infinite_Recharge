package frc.robot.subsystem.scoring.shooter;

import java.util.Arrays;
import java.util.List;

import frc.robot.subsystem.vision.VisionSubsystem;

import frc.robot.utils.math.MathUtils;
import frc.robot.config.Config;

public class ShooterCalculator {

    private final List<VelocityPoint> points;
    VisionSubsystem visionSubsystem;
    SplineVelocityPoint splineVPoint;

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
     */
    public ShooterCalculator() {
        this(Arrays.asList(
            new VelocityPoint(109, 5000, 45), // initiation line
            new VelocityPoint(130, 5000, 45.5),
            new VelocityPoint(150, 5000, 45),
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

    // TODO: empirically test this
    public double calculateSpeed_rpm() {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);

            return splineVPoint.getSpeedSpline(distance);
        }
        // calculate the velocity for this distance_in
        return 0;
    }

    public double calculateSpeed_ticks() {
        double speed_rpm = calculateSpeed_rpm();
        double speed_ticks = MathUtils.unitConverter(speed_rpm, 600, config.shooter.shooter.ticksPerRevolution) * config.shooter.shooterGearRatio;

        return speed_ticks;
    }

    // TODO: empirically test this
    // decreases as u get closer
    // increases as u get it farther
    public double calculateHoodAngle_deg() {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);

            return splineVPoint.getAngleSpline(distance);
        }
        // calculate the hood angle for this distance_in
        return 0;
    }

}
