package frc.robot.subsystem.vision;

public class VelocityPoint implements Comparable<VelocityPoint> {
    private final double distance_in;
    private final double speed_rpm;
    private final double elevationAngle_deg;

    public VelocityPoint(double d, double s, double eAngle) {
        distance_in = d;
        speed_rpm = s;
        elevationAngle_deg = eAngle;
    }

    public double getDistance_in() {
        return distance_in;
    }

    public double getSpeed_rpm() {
        return speed_rpm;
    }

    public double getElevationAngle_deg() {
        return elevationAngle_deg;
    }

    @Override
    public int compareTo(VelocityPoint other) {
        return Double.compare(this.distance_in, other.distance_in);
    }
}