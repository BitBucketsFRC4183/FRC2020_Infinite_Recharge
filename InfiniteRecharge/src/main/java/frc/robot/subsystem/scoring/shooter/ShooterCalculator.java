package frc.robot.subsystem.scoring.shooter;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import frc.robot.subsystem.vision.VisionSubsystem;

public class ShooterCalculator {

    private final List<VelocityPoint> points;
    VisionSubsystem visionSubsystem;

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
            new VelocityPoint(10, 40, 10), 
            new VelocityPoint(20, 50, 20)
            )
        );
    }

    // TODO: empirically test this
    public double calculateVelocity_in(final double distance_in) {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);

            int closestIndex = Collections.binarySearch(points, new VelocityPoint(distance_in));
        }
        // calculate the velocity for this distance_in
        return 0;
    }

    // TODO: empirically test this
    // decreases as u get closer
    // increases as u get it farther
    public double calculateHoodAngle_deg(final double distance_in) {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double ty = visionSubsystem.getTy();
            double distance = visionSubsystem.approximateDistanceFromTarget(ty);

            int closestIndex = Collections.binarySearch(points, new VelocityPoint(distance_in));
        }
        // calculate the hood angle for this distance_in
        return 0;
    }

}
