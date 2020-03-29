/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.navigation;

/**
 * Add your docs here.
 */
public class NavigationConstants {
    // amount of data to keep for testing things like variance of gyroscope and accelerometer values
    public static final int IMU_DATA_SIZE = 50;

    // x and y positions of the power port on the field, aka the LimeLight target
    public static final double TARGET_X = 1.700784;
    public static final double TARGET_Y = 7.9915;

    public static final double IN_TO_M = 254 / 10000.0;

    public static final double KALMAN_FILTER_UPDATE_RATE_S = 0.02;
}
