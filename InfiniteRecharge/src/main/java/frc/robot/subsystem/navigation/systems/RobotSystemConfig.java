package frc.robot.subsystem.navigation.systems;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

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
}