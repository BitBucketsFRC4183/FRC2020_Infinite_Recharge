/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.operatorinterface.OI;
import frc.robot.operatorinterface.PS4Constants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.scoring.intake.IntakeSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Config config;

  public float deltaTime;
  public long currentTime;
  public long lastTime;

  private final OI oi = new OI();

  private NavigationSubsystem navigationSubsystem;
  private DriveSubsystem driveSubsystem;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    config = ConfigChooser.getConfig();

    navigationSubsystem = new NavigationSubsystem(config);
    navigationSubsystem.initialize();

    driveSubsystem = new DriveSubsystem(config, navigationSubsystem, oi);
    driveSubsystem.initialize();

    shooterSubsystem = new ShooterSubsystem(config);
    shooterSubsystem.initialize();

    intakeSubsystem = new IntakeSubsystem(config);
    intakeSubsystem.initialize();

    lastTime = System.currentTimeMillis();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    currentTime = System.currentTimeMillis();
    deltaTime = (currentTime - lastTime) / 1000f;
    SmartDashboard.putNumber("deltaTime", deltaTime);

    driveSubsystem.periodic(deltaTime);
    shooterSubsystem.periodic(deltaTime);
    intakeSubsystem.periodic(deltaTime);
    CommandScheduler.getInstance().run();

    lastTime = currentTime;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //////////////////////////////////////////////////////////////////////////////
    // Drive Subsystem

    driveSubsystem.setDriverRawSpeed(oi.speed());
    driveSubsystem.setDriverRawTurn(oi.turn());

    //////////////////////////////////////////////////////////////////////////////
    // Intake Subsystem

    // Intake on pressing circle.
    if (oi.driverControl.getRawButton(PS4Constants.CIRCLE.getValue())) {
      intakeSubsystem.intake();
    } else {
      intakeSubsystem.doNotIntake();
    }

    //////////////////////////////////////////////////////////////////////////////
    // Shooter Subsystem

    // Shoot on pressing square.
    if (oi.driverControl.getRawButton(PS4Constants.SQUARE.getValue())) {
      shooterSubsystem.shoot();
    } else {
      shooterSubsystem.doNotShoot();
    }

    // Rotate the turret with the joystick.
    if (oi.driverControl.getRawButton(PS4Constants.TRIANGLE.getValue())) {
      shooterSubsystem.rotate(oi.driverControl.getRawAxis(PS4Constants.LEFT_STICK_X.getValue()));
    } else {
      shooterSubsystem.rotate(0);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static Robot win() {
    return new Robot();
  }
}
