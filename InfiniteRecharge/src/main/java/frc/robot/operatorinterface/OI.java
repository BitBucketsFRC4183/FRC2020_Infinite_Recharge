package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystem.drive.DriveConstants;

public class OI {

    static final int DRIVER_JOYSTICK_ID = 0;
    static final int OPERATOR_JOYSTICK_ID = 1;

    // TODO: Make a get/set function instead of setting to public
    public final Joystick driverControl = new Joystick(DRIVER_JOYSTICK_ID);
    public final Joystick operatorControl = new Joystick(OPERATOR_JOYSTICK_ID);

    // ****************************
    // AXIS DEFINITIONS
    // ****************************
    static final int DRIVE_SPEED_AXIS = PS4Constants.LEFT_STICK_Y.getValue();
    static final int DRIVE_TURN_AXIS = PS4Constants.RIGHT_STICK_X.getValue();
    static final int OPERATOR_MANUAL_SPINUP_AXIS = PS4Constants.LEFT_TRIGGER.getValue();
    static final int OPERATOR_MANUAL_AZIMUTH_AXIS = PS4Constants.RIGHT_STICK_X.getValue();
    static final int OPERATOR_MANUAL_ELEVATION_AXIS = PS4Constants.RIGHT_STICK_Y.getValue();
    static final int OPERATOR_MANUAL_CLIMB_LEFT = PS4Constants.LEFT_STICK_Y.getValue();
    static final int OPERATOR_MANUAL_CLIMB_RIGHT = PS4Constants.RIGHT_STICK_Y.getValue();

    static final int PIT_REWIND_CLIMB_LEFT = PS4Constants.LEFT_STICK_Y.getValue();
    static final int PIT_REWIND_CLIMB_RIGHT = PS4Constants.RIGHT_STICK_Y.getValue();

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
    static final int DRIVE_LOW_SENSITIVE_BUTTON = PS4Constants.R1.getValue();
    static final int DRIVE_INVERT_BUTTON = PS4Constants.L1.getValue();
    static final int DRIVE_AUTO_AIM = PS4Constants.CIRCLE.getValue();
    static final int DRIVE_ALIGN_LOCK_BUTTON = PS4Constants.SHARE.getValue();
    static final int DRIVE_LOCK_BUTTON = PS4Constants.OPTIONS.getValue();
    static final int DRIVE_AUTO_ALIGN = PS4Constants.CROSS.getValue();
    static final int DRIVE_METHOD_SWITCH_BUTTON = PS4Constants.LEFT_TRIGGER.getValue();
    static final int DRIVER_CLIMB_ACTIVATE = PS4Constants.PS4.getValue();
    // static final int DRIVER_ROTATION_CONTROL = PS4Constants.TRIANGLE.getValue();
    // static final int DRIVER_COLOR_CONTROL = PS4Constants.CROSS.getValue();

    static final int OPERATOR_SPINNYBOI_FORWARD = PS4Constants.TRIANGLE.getValue();
    static final int OPERATOR_SPINNYBOI_BACKWARD = PS4Constants.CROSS.getValue();
    static final int OPERATOR_INTAKE_IN_POV = 180;
    static final int OPERATOR_INTAKE_OUT_POV = 0;
    static final int OPERATOR_INTAKE_TOGGLE = PS4Constants.R1.getValue();
    static final int OPERATOR_NEXT_POSITION_ELEVATION_POV = 90;
    static final int OPERATOR_LAST_POSITION_ELEVATION_POV = 270;
    static final int OPERATOR_SPINUP = PS4Constants.L1.getValue();
    static final int OPERATOR_AUTO_AIM = PS4Constants.R2.getValue();
    static final int OPERATOR_FIRE = PS4Constants.CIRCLE.getValue();
    static final int OPERATOR_TURRET_ZERO = PS4Constants.SQUARE.getValue();
    static final int OPERATOR_SET_ELEVATION_TO_DASHBOARD_NUMBER = PS4Constants.OPTIONS.getValue();
    static final int OPERATOR_CLIMB_ACTIVATE = PS4Constants.PS4.getValue();
    static final int OPERATOR_FEEDER = PS4Constants.L2.getValue();

    // forced Idle for corresponding subsystems
    static final int DRIVER_IDLE = PS4Constants.TRACKPAD.getValue();
    static final int OPERATOR_IDLE = PS4Constants.TRACKPAD.getValue();

    public double manualSpinupAxis() {
        return operatorControl.getRawAxis(OPERATOR_MANUAL_SPINUP_AXIS);
    }

    public double manualElevationAxis() {
        return -operatorControl.getRawAxis(OPERATOR_MANUAL_ELEVATION_AXIS);
    }

    public double pitRewindLeft() {
        return operatorControl.getRawAxis(PIT_REWIND_CLIMB_LEFT);
    }

    public double pitRewindRight() {
        return operatorControl.getRawAxis(PIT_REWIND_CLIMB_RIGHT);
    }

    public double manualAzimuthAxis() {
        return operatorControl.getRawAxis(OPERATOR_MANUAL_AZIMUTH_AXIS);
    }
    public double manualClimbLeft() {
        return operatorControl.getRawAxis(OPERATOR_MANUAL_CLIMB_LEFT);
    }
    public double manualClimbRight() {
        return operatorControl.getRawAxis(OPERATOR_MANUAL_CLIMB_RIGHT);
    }

    public boolean feeder() {
        return operatorControl.getRawButton(OPERATOR_FEEDER);
    }

    public boolean nextPositionElevation() {
        return operatorControl.getPOV() == OPERATOR_NEXT_POSITION_ELEVATION_POV;
    }

    public boolean lastPositionElevation() {
        return operatorControl.getPOV() == OPERATOR_LAST_POSITION_ELEVATION_POV;
    }

    public boolean setElevationToDashboardNumber() {
        return operatorControl.getRawButton(OPERATOR_SET_ELEVATION_TO_DASHBOARD_NUMBER);
    }

    public boolean zero() {
        return operatorControl.getRawButton(OPERATOR_TURRET_ZERO);
    }

    public boolean operatorAimBot() {
        return operatorControl.getRawButton(OPERATOR_AUTO_AIM);
    }

    public boolean driveAimBot() {
        return driverControl.getRawButton(DRIVE_AUTO_AIM);
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

    public boolean intaking() {
        return operatorControl.getPOV() == OPERATOR_INTAKE_IN_POV;
    }

    public boolean outaking() {
        return operatorControl.getPOV() == OPERATOR_INTAKE_OUT_POV;
    }

    public boolean barDownButtonPressed() {
        return operatorControl.getRawButtonPressed(OPERATOR_INTAKE_TOGGLE);
    }

    // public boolean rotationControl() {
    //     return driverControl.getRawButtonPressed(DRIVER_ROTATION_CONTROL);
    // }

    // public boolean colorControl() {
    //     return driverControl.getRawButtonPressed(DRIVER_COLOR_CONTROL);
    // }

    public boolean spinnyBoiForward(){
        return operatorControl.getRawButtonPressed(OPERATOR_SPINNYBOI_FORWARD);
    }

    public boolean spinnyBoiBackward(){
        return operatorControl.getRawButtonPressed(OPERATOR_SPINNYBOI_BACKWARD);
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
        return driverControl.getRawButton(DRIVE_METHOD_SWITCH_BUTTON);
    }

    public boolean climbActivate() {
        return operatorControl.getRawButtonPressed(OPERATOR_CLIMB_ACTIVATE);
    }
}
