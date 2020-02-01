package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystem.drive.DriveConstants;

public class OI {

	private final int DRIVER_JOYSTICK_ID = 0;
	private final int OPERATOR_JOYSTICK_ID = 1;

	// TODO: Make a get/set function instead of setting to public
	public final Joystick driverControl = new Joystick(DRIVER_JOYSTICK_ID);
	public final Joystick operatorControl = new Joystick(OPERATOR_JOYSTICK_ID);

	// ****************************
	// AXIS DEFINITIONS
	// ****************************
	private final int DRIVE_SPEED_AXIS = PS4Constants.LEFT_STICK_Y.getValue();
    private final int DRIVE_TURN_AXIS = PS4Constants.RIGHT_STICK_X.getValue();
    private final int OPERATOR_MANUAL_SPINUP_AXIS = PS4Constants.LEFT_TRIGGER.getValue();
    private final int OPERATOR_MANUAL_AZIMUTH_AXIS = PS4Constants.RIGHT_STICK_X.getValue();
    private final int OPERATOR_MANUAL_ELEVATION_AXIS = PS4Constants.RIGHT_STICK_Y.getValue();



	public double speed() {
		// Default to -1 to make up-stick positive because raw up-stick is negative
		return (invertDrive() ? 1.0 : -1.0) * driverControl.getRawAxis(DRIVE_SPEED_AXIS);
	}

	/**
	 * turn - returns a turn rate command from the driver joystick in the normal
	 * range [-1,1]
	 */
	public double turn() {
		// Defult to -1 because right stick is naturally negative and we want that to
		// really mean positive rate turn. Since traditional motion mechanics uses a
		// RPY or NED reference frame where Y (yaw) or D (down) are both positive
		// "down", right hand coordinate rules dictate that positive rotations are to
		// the right (i.e. vectors R x P = Y and N x E = D)
		return DriveConstants.TURN_SIGN * driverControl.getRawAxis(DRIVE_TURN_AXIS);
	}

	// ****************************
	// BUTTON DEFINITIONS
	// ****************************
	private final int DRIVE_LOW_SENSITIVE_BUTTON = PS4Constants.R1.getValue();
	private final int DRIVE_INVERT_BUTTON = PS4Constants.L1.getValue();
	private final int DRIVE_ALIGN_LOCK_BUTTON = PS4Constants.SHARE.getValue();
	private final int DRIVE_LOCK_BUTTON = PS4Constants.OPTIONS.getValue();
	private final int DRIVE_AUTO_ALIGN = PS4Constants.CROSS.getValue();
	private final int INTAKE_IN_POV = 0;
    private final int INTAKE_OUT_POV = 180; 
    private final int OPERATOR_SPINUP = PS4Constants.R2.getValue();
    private final int OPERATOR_AUTO_AIM = PS4Constants.L1.getValue();
    private final int OPERATOR_FIRE = PS4Constants.CIRCLE.getValue();

	// forced Idle for corresponding subsystems
	private final int DRIVER_IDLE = PS4Constants.TRACKPAD.getValue();
    private final int OPERATOR_IDLE = PS4Constants.TRACKPAD.getValue();
    public double manualSpinupAxis() {
		return operatorControl.getRawAxis(OPERATOR_MANUAL_SPINUP_AXIS);
	}
    
    public double manualElevationAxis() {
		return operatorControl.getRawAxis(OPERATOR_MANUAL_ELEVATION_AXIS);
	}
    
    public double manualAzimuthAxis() {
		return operatorControl.getRawAxis(OPERATOR_MANUAL_AZIMUTH_AXIS);
	}

    public boolean aimBot() {
		return operatorControl.getRawButton(OPERATOR_AUTO_AIM);
	}
    
    public boolean fire() {
		return operatorControl.getRawButton(OPERATOR_FIRE);
	}
    
    public boolean spinUp() {
		return operatorControl.getRawButton(OPERATOR_SPINUP);
	}

	public boolean lowSpeed() {
		return driverControl.getRawButton(DRIVE_LOW_SENSITIVE_BUTTON);
	}

	public boolean intaking(){
		return operatorControl.getPOV() == INTAKE_IN_POV;
	}
	public boolean outaking(){
		return operatorControl.getPOV() == INTAKE_OUT_POV;
	}

	/**
	 * invertDrive - a private function used by speed() to invert the speed joystick
	 * sense temporarily
	 */
	private boolean invertDrive() {
		return driverControl.getRawButton(DRIVE_INVERT_BUTTON);
	}

	/**
	 * alighLock - indicates driver would like to have some help driving straight or
	 * otherwise resist a turn
	 */
	public boolean alignLock() {
		return driverControl.getRawButton(DRIVE_ALIGN_LOCK_BUTTON);
	}

	/**
	 * driveLock - indicates driver would like to have some help holding position
	 * The intent is to use the drive system position control loops to act like a
	 * brake.
	 */
	public boolean driveLock() {
		return driverControl.getRawButton(DRIVE_LOCK_BUTTON);
	}

	public boolean rotationToVelocity() {
		return false;
	}
}
