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

    public static final double MAX_ALLOWED_SPEED_IPS = 8.0*12.0;
    public static final double MAX_ALLOWED_TURN_DPS  = 180.0;
    public static final double MAX_ALLOWED_TURN_RADPS = Math.toRadians(MAX_ALLOWED_TURN_DPS);
    public static final double STANDARD_G_FTPSPS = 32.1740;
    public static final double MAX_LAT_ACCELERATION_IPSPS = STANDARD_G_FTPSPS * 12.0;

    // TODO
    public static final double WHEEL_TRACK_INCHES = 12*1.53;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI*WHEEL_DIAMETER_INCHES;

    // Identify what type of feedback device we will use on this drive base
    // Assume that all feedback devices are the same type on all axels that
    // need to be measured.
    // TODO
    public static final FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;

    // Speed constants
    public static final double DRIVE_MOTOR_NATIVE_TICKS_PER_REV = 8192;

    // The motor controllers we use (TalonSRX) return velocity in terms of native ticks per 100 ms 
    // and expect commands to be similarly dimensioned.
    // Even though this is a constants package, we provide the convenient conversion to/from
    // inches per second to the native ticks per 100 ms.
    // NOTE: Integer truncation is assumed for a maximum reduction of 10 ticks per second.
    // For 8192 ticks per rev that is error of < 0.07 RPM
    public static int ipsToTicksP100(double ips)
    {
        double rps = ips / WHEEL_CIRCUMFERENCE_INCHES;
        return (int)(DRIVE_MOTOR_NATIVE_TICKS_PER_REV * rps / 10.0);
    }
    public static double ticksP100ToIps(int ticksP100)
    {
        double rps = ticksP100 * 10.0 / DRIVE_MOTOR_NATIVE_TICKS_PER_REV;
        return rps * WHEEL_CIRCUMFERENCE_INCHES;
    }

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
    public static final double MAX_VOLTS = 11.9; // charge your batteries!!!

    public static final double KS_VOLTS = 1.63;

    public static final double KV_VOLT_S_PER_FT = 0.521;
    // 1/ft * 1ft/12in * 1 rad/radius in
    public static final double KV_VOLT_S_PER_RAD = KV_VOLT_S_PER_FT / 12 / (WHEEL_DIAMETER_INCHES / 2);
    // 1V = (1V/VOLTAGE) * VOLTAGE
    // 1V = (1V/VOLTAGE) * 1023 voltage increments
    // 1V * 1023 voltage increments / (battery voltage) volts
    // S * 10 (100ms) / S
    // rad * 1 rev/(2pi rad) * EPR ticks/rev
    public static final double KV_100MS_PER_TICK /* WHY */ = KV_VOLT_S_PER_RAD * 10 / (2 * Math.PI) / DRIVE_MOTOR_NATIVE_TICKS_PER_REV / MAX_VOLTS;

    public static final double KP_VOLT_S_PER_FT = 0.00332;
    public static final double KP_VOLT_S_PER_RAD = KP_VOLT_S_PER_FT / 12 / (WHEEL_DIAMETER_INCHES / 2);
    public static final double KP_100MS_PER_TICK_PERCENT /* WHY */ = KP_VOLT_S_PER_RAD * 10 / (2 * Math.PI) / DRIVE_MOTOR_NATIVE_TICKS_PER_REV / MAX_VOLTS;



    public static final double ROTATION_DRIVE_MAX_OFFSET_DEG = 45;
    public static final double ROTATION_DRIVE_KP = 5*2*Math.PI/360;
}