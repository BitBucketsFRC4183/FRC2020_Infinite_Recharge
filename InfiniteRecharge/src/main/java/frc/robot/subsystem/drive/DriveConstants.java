/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/**
 * Add your docs here.
 */
public class DriveConstants {
    public static final double JOYSTICK_DEADBAND = 0.15;

    // Set velocity follower type to false when independent gear boxes are being used
    // Set to true of all wheels on one side are physically linked
    public static final boolean CLOSED_LOOP_FOLLOWER = false;

    // TODO: do we want to use these?
    public static final double DRIVE_MOTOR_OPEN_LOOP_RAMP_SEC   = 0.750;	// Second from neutral to full (easy on the gears)
    // Closed loop ramp rates are tricky; too much and the PID can become unstable as
    // if there was a lot of system lag; we must be cautious!
    // An alternative is to use an alpha filter on the inputs to prevent the user
    // from changing the command too rapidly
    public static final double DRIVE_MOTOR_CLOSED_LOOP_RAMP_SEC = 0.4;	    // No ramp rate on closed loop (use Motion Magic)

    public static final double STANDARD_G_FTPSPS = 32.1740;
    public static final double MAX_LAT_ACCELERATION_IPSPS = STANDARD_G_FTPSPS * 1200.0;

    // Identify what type of feedback device we will use on this drive base
    // Assume that all feedback devices are the same type on all axels that
    // need to be measured.
    // TODO
    public static final FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;

    // FIRST !!! get the sensor phase correct.
    // If positive input to motor controller (green LED) makes the sensor
    // return positive increasing counts then the sensor phase is set correctly.
    // E.g., start with false, if the counts go the correct direction you are good
    // to go; if not, set the flag to true (indicating the sensor inverted from the
    // positive input).
    public static final boolean LEFT_DRIVE_MOTOR_SENSOR_PHASE = true;
    public static final boolean RIGHT_DRIVE_MOTOR_SENSOR_PHASE = true;

    
    // Define some constants for using the motor controllers
    // TODO: move this to a common motor class or utility
	public static final int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response
    
    public static final int SUPER_HIGH_STATUS_FRAME_PERIOD_MS  =   5;	// CAUTION!
	public static final int HIGH_STATUS_FRAME_PERIOD_MS        =  10;	
	public static final int MEDIUM_HIGH_STATUS_FRAME_PERIOD_MS =  20;
	public static final int MEDIUM_STATUS_FRAME_PERIOD_MS      =  50;
    public static final int LOW_STATUS_FRAME_PERIOD_MS         = 100;

    

    public static final double TURN_SIGN = 1.0;



    // not sure if we can use Double.POSITIVE_INFINITY so just be safe
    public static final double MAX_LIN_ACCELERATION_IPSPS = 999999999;

    public static final double ROTATION_DRIVE_RANGE_DEG = 20;



    /** for voltage compensation */
    // TODO: add to config
    public static final double MAX_VOLTS = 11.9; // charge your batteries!!!

    public static final double KS_VOLTS = 1.63;

    public static final double KV_VOLT_S_PER_FT = 0.521;



    public static final double ROTATION_DRIVE_MAX_OFFSET_DEG = 45;



    public static final double METERS_PER_INCH = 254 / 10000.0; // haha 254



    static final double AUTO_MAX_VOLTAGE = 11;



    public static final double AUTO_ALIGN_KP = 5 * Math.PI / 180;
    public static final double AUTO_ALIGN_KI = 0.01;
    public static final double AUTO_ALIGN_KD = 0;
}