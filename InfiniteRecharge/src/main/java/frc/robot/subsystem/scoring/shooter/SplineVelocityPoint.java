package frc.robot.subsystem.scoring.shooter;

import java.util.List;
import frc.robot.utils.math.interpolation.spline.Spline;
import frc.robot.subsystem.scoring.shooter.ShooterCalculator.VelocityPoint;

public class SplineVelocityPoint {
    private final Spline speedSpline;
    private final Spline angleSpline;

/**
 * Given an arraylist of velocity points, it calculates splines for velocity and distance for a given distance
 * @param vPoints Arraylist of velocity points, used to calculate the spline (defined in ShooterCalculator.java)
 */
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