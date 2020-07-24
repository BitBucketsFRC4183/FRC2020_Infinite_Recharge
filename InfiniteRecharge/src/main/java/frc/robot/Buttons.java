package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.operatorinterface.PS4Constants;

public class Buttons {

    Joystick driverControl = new Joystick(0);
    Joystick operatorControl = new Joystick(1);

    // drive
    JoystickButton driveAutoAim = new JoystickButton(driverControl, PS4Constants.CIRCLE.getId());

    int driveSpeedAxis = PS4Constants.LEFT_STICK_Y.getId();
    int driveTurnAxis = PS4Constants.RIGHT_STICK_X.getId();

    // intake
    POVButton operatorIntake = new POVButton(operatorControl, 180); // down
    POVButton operatorOutake = new POVButton(operatorControl, 0); // up
    JoystickButton operatorToggleIntake = new JoystickButton(operatorControl, PS4Constants.R1.getId());

    // climb
    JoystickButton operatorActivateClimb = new JoystickButton(operatorControl, PS4Constants.PS4.getId());
    JoystickButton driveActivateClimb = new JoystickButton(driverControl, PS4Constants.PS4.getId());

    int operatorClimbLeft = PS4Constants.LEFT_STICK_Y.getId();
    int operatorClimbRight = PS4Constants.RIGHT_STICK_Y.getId();
  
    // shooter
    JoystickButton operatorAimBot = new JoystickButton(operatorControl, PS4Constants.R2.getId());
    JoystickButton operatorSpinUp = new JoystickButton(operatorControl, PS4Constants.L1.getId());
    JoystickButton operatorFeeder = new JoystickButton(operatorControl, PS4Constants.L2.getId());
    JoystickButton operatorFire = new JoystickButton(operatorControl, PS4Constants.CIRCLE.getId());
    JoystickButton operatorZero = new JoystickButton(operatorControl, PS4Constants.SQUARE.getId());

    int operatorAzimuth = PS4Constants.RIGHT_STICK_X.getId();
    int operatorElevation = PS4Constants.RIGHT_STICK_Y.getId();

    POVButton operatorNextElevation = new POVButton(operatorControl, 90); // right
    POVButton operatorLastElevation = new POVButton(operatorControl, 270); // left

    JoystickButton setElevationToDashboardNum = new JoystickButton(operatorControl, PS4Constants.OPTIONS.getId());

    // spinnyboi
    JoystickButton operatorSpinForward = new JoystickButton(operatorControl, PS4Constants.TRIANGLE.getId());
    JoystickButton operatorSpinBackward = new JoystickButton(operatorControl, PS4Constants.CROSS.getId());
    

    /////////// unimplemented

    JoystickButton driverRotationControl = new JoystickButton(driverControl, PS4Constants.TRIANGLE.getId());
    JoystickButton driverColorControl = new JoystickButton(driverControl, PS4Constants.CROSS.getId());

    // forced Idle for corresponding subsystems (still unimplemented)
    JoystickButton driverIdle = new JoystickButton(driverControl, PS4Constants.TRACKPAD.getId());
    JoystickButton operatorIdle = new JoystickButton(operatorControl, PS4Constants.TRACKPAD.getId());

}