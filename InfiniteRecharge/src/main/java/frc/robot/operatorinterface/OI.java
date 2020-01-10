package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystem.drive.DriveConstants;

public class OI {
	// Only use the xbox joystick if we explicitly request it.
	private ControllerMapper controllerMapper;
	


	public OI() {
		if (System.getProperty("xbox", null) != null) {
			controllerMapper = ControllerMapper.xbox();
		} else {
			controllerMapper = ControllerMapper.ps4();
		}
	}



	private final int DRIVER_JOYSTICK_ID = 0;
	private final int OPERATOR_JOYSTICK_ID = 1;

	// TODO: Make a get/set function instead of setting to public
	public final Joystick driverControl = new Joystick(DRIVER_JOYSTICK_ID);
	public final Joystick operatorControl = new Joystick(OPERATOR_JOYSTICK_ID);

	// ****************************
	// AXIS DEFINITIONS
	// ****************************
	private final int DRIVE_SPEED_AXIS = controllerMapper.getLeftStickY();
	private final int DRIVE_TURN_AXIS = controllerMapper.getRightStickX();



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
	private final int DRIVE_LOW_SENSITIVE_BUTTON = controllerMapper.getR1();
	private final int DRIVE_INVERT_BUTTON = controllerMapper.getL1();
	private final int DRIVE_ALIGN_LOCK_BUTTON = controllerMapper.getShare();
	private final int DRIVE_LOCK_BUTTON = controllerMapper.getOption();
	private final int DRIVE_AUTO_ALIGN = controllerMapper.getCross();

	// forced Idle for corresponding subsystems
	private final int DRIVER_IDLE = controllerMapper.getTrackpad();
	private final int OPERATOR_IDLE = controllerMapper.getTrackpad();

	public boolean lowSpeed() {
		return driverControl.getRawButton(DRIVE_LOW_SENSITIVE_BUTTON);
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
