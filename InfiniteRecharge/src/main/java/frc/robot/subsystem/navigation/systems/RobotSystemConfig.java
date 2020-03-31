package frc.robot.subsystem.navigation.systems;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import frc.robot.subsystem.navigation.NavigationConstants;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class RobotSystemConfig {
    public DoubleSupplier azimuthDegrees;
    public DoubleFunction<Double> tyFromDistance;
    public DoubleSupplier leftVolts;
    public DoubleSupplier rightVolts;



    // assume 5inch error is 2 standard deviations away
    public final double stdev = (5.0 / Math.sqrt(2)) / 2 * NavigationConstants.IN_TO_M;
    public final double x0_var = stdev * stdev;
    public final double y0_var = stdev * stdev;

    public final double theta_var = Math.pow(2 * Math.PI / 180, 2); // 2 deg standard deviation

    public final double velocity_var = 0.000000000001; // 0 won't let you take a Cholesky decomposition :(

    public final double ll_angle_var = Math.pow(2 * Math.PI / 180, 2); // wild guess lol
    public final double imu_yaw_var = Math.pow(0.2 * Math.PI / 180, 2); // another wild guess
}