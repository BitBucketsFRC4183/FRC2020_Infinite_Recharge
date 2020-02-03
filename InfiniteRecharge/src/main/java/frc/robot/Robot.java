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
import frc.robot.subsystem.spinnyboi.SpinnyBoiSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
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
    private SpinnyBoiSubsystem spinnyBoiSubsystem;
    private NavigationSubsystem navigationSubsystem;
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private Config config;

    public float deltaTime;
    public long currentTime;
    public long lastTime;

    private final OI oi = new OI();

    

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        config = ConfigChooser.getConfig();

        visionSubsystem = new VisionSubsystem(config);
        visionSubsystem.initialize();

        navigationSubsystem = new NavigationSubsystem(config, visionSubsystem);
        navigationSubsystem.initialize();

        driveSubsystem = new DriveSubsystem(config, navigationSubsystem, oi);
        driveSubsystem.initialize();

        shooterSubsystem = new ShooterSubsystem(config, visionSubsystem);
        shooterSubsystem.initialize();

        intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        spinnyBoiSubsystem = new SpinnyBoiSubsystem(config);
        spinnyBoiSubsystem.initialize();

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

        double t0 = System.nanoTime();
        driveSubsystem.periodic(deltaTime);
        double t1 = System.nanoTime();
        shooterSubsystem.periodic(deltaTime);
        double t2 = System.nanoTime();
        intakeSubsystem.periodic(deltaTime);
        double t3 = System.nanoTime();
        navigationSubsystem.periodic(deltaTime);
        double t4 = System.nanoTime();
        spinnyBoiSubsystem.periodic(deltaTime);
        double t5 = System.nanoTime();
        //System.out.println("Drive: " + (t1 - t0) / 1000000 + "ms");
        //System.out.println("Shooter: " + (t2 - t1) / 1000000 + "ms");
        //System.out.println("Intake: " + (t3 - t2) / 1000000 + "ms");
        //System.out.println("Navigation: " + (t4 - t3) / 1000000 + "ms");
        //System.out.println("Spinny Boi: " + (t5 - t4) / 1000000 + "ms");

        SmartDashboard.putNumber("periodic time", (t5 - t0) / 1000000);

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
        if (oi.intaking()) {
            intakeSubsystem.intake();
        } else if (oi.outaking()) {
            intakeSubsystem.outake();
        } else {
            intakeSubsystem.off();
        }

        // Pivot Intake Bar
        if (oi.barDownButtonPressed()) {
            intakeSubsystem.toggleIntakeArm();
        }

        //////////////////////////////////////////////////////////////////////////////
        // Shooter Subsystem

        SmartDashboard.putNumber("BallManagementSubsystem/Output Percent", 50);

        // Spin up on pressing [spinUp]
        if (oi.spinUp()) {
            shooterSubsystem.spinUp();
        } else {
            shooterSubsystem.stopSpinningUp();
        }

        // Fire on pressing [fire]
        if (oi.fire()) {
            shooterSubsystem.fire();
        } else {
            shooterSubsystem.holdFire();
        }

        // Rotate the turret with [manualAzimuthAxis]
        if (Math.abs(oi.manualAzimuthAxis()) >= config.shooter.manualAzimuthDeadband || Math.abs(oi.manualElevationAxis()) >= config.shooter.manualElevationDeadband) {
            shooterSubsystem.rotate(oi.manualAzimuthAxis(), oi.manualElevationAxis());
        } else {
            shooterSubsystem.rotate(0, 0);
        }

        if (oi.aimBot()) {
            shooterSubsystem.autoAim();
        }

        if (oi.zero()) {
            shooterSubsystem.rotateToDeg(0, 0);
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    // COMMANDS the robot to WIN!
    public static Robot win() {
        return new Robot();
    }

    public static Robot beat254() {
        return win();
    }
}