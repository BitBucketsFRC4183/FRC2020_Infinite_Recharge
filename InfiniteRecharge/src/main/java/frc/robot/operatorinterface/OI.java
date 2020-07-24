package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystem.drive.DriveConstants;

public class OI {

    public final Joystick driverControl = new Joystick(0);
    public final Joystick operatorControl = new Joystick(1);

    // ****************************
    // AXIS DEFINITIONS
    // ****************************
    static final int OPERATOR_MANUAL_SPINUP_AXIS = PS4Constants.LEFT_TRIGGER.getId();

    // ****************************
    // BUTTON DEFINITIONS
    // ****************************
    static final int DRIVE_LOW_SENSITIVE_BUTTON = PS4Constants.R1.getId();
    static final int DRIVE_INVERT_BUTTON = PS4Constants.L1.getId();
    static final int DRIVE_ALIGN_LOCK_BUTTON = PS4Constants.SHARE.getId();
    static final int DRIVE_LOCK_BUTTON = PS4Constants.OPTIONS.getId();
    static final int DRIVE_AUTO_ALIGN = PS4Constants.CROSS.getId();
    static final int DRIVE_METHOD_SWITCH_BUTTON = PS4Constants.LEFT_TRIGGER.getId();

    public double manualSpinupAxis() {
        return operatorControl.getRawAxis(OPERATOR_MANUAL_SPINUP_AXIS);
    }


    public boolean lowSpeed() {
        return driverControl.getRawButton(DRIVE_LOW_SENSITIVE_BUTTON);
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
        return driverControl.getRawButton(DRIVE_METHOD_SWITCH_BUTTON);
    }

}
