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
    public static final double WHEEL_TRACK_INCHES = 23.5;
    public static final double WHEEL_DIAMETER_INCHES = 6.0;
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

    public static final int PRIMARY_PID_LOOP  = 0; // Constants to support new Talon interface types
	public static final int CASCADED_PID_LOOP = 1; // That should have been enumerated rather than int
	public static final int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response
    
    public static final int SUPER_HIGH_STATUS_FRAME_PERIOD_MS  =   5;	// CAUTION!
	public static final int HIGH_STATUS_FRAME_PERIOD_MS        =  10;	
	public static final int MEDIUM_HIGH_STATUS_FRAME_PERIOD_MS =  20;
	public static final int MEDIUM_STATUS_FRAME_PERIOD_MS      =  50;
    public static final int LOW_STATUS_FRAME_PERIOD_MS         = 100;



    // These Motion Magic values defined the shape of the trapezoidal profile for speed
    // The cruise speed is the maximum speed during the profile and is chosen to keep
    // below the maximum (which varies with battery voltage). The acceleration is the
    // slope allowed to reach the cruise speed or zero (hence, a trapezoid)
    //
    // Setting this to 80% of maximum is a reasonable place to start;
    // However, the acceleration is currently default to reach cruising speed within 1 second 
    // and may need to be increased or decreased depending on static friction limits of tires
        
    public static final int DRIVE_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS = 0; // TODO
    public static final int DRIVE_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS = (int) (4346/1.5); // 0.26 g on wood//DRIVE_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS;

    

	public static final double TURN_SIGN = 1.0;
}