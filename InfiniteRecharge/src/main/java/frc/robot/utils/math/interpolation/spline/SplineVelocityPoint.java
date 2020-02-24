package frc.robot.utils.math.interpolation.spline;

import java.util.List;
import frc.robot.subsystem.scoring.shooter.ShooterCalculator.VelocityPoint;

public class SplineVelocityPoint {
    private final Spline speedSpline;
    private final Spline angleSpline;

    public SplineVelocityPoint(List<VelocityPoint> vPoints) {
        double[] distanceX =  new double[vPoints.size()];
        double[] speedY = new double[vPoints.size()];
        double[] angleY = new double[vPoints.size()];

        for (int i = 0; i < vPoints.size(); i++) {
            distanceX[i] = vPoints.get(i).getDistance_in();
            speedY[i] = vPoints.get(i).getSpeed_rpm();
            angleY[i] = vPoints.get(i).getHoodAngle_deg();
        }

        speedSpline = new Spline(distanceX, speedY);
        angleSpline = new Spline(distanceX, angleY);
    }

    public double getSpeedSpline(double x) {
        return speedSpline.get(x);
    }

    public double getAngleSpline(double x) {
        return angleSpline.get(x);
    }
}