/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.subsystem.spinnyboi.SpinnyBoiSubsystem;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.climber.ClimbSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.ProxyCommand;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.drive.DriveUtils;
import frc.robot.subsystem.drive.auto.AutoAlign;
import frc.robot.subsystem.drive.auto.AutoDrive;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.pidhelper.PIDHelperSubsystem;
import frc.robot.subsystem.scoring.intake.IntakeSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

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
    private ClimbSubsystem climbSubsystem;
    private Config config;

    public float deltaTime;
    public long currentTime;
    public long lastTime;

    @Log.Exclude
    private List<BitBucketSubsystem> subsystems = new ArrayList<>();

    private CANChecker canChecker;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        config = ConfigChooser.getConfig();

        visionSubsystem = new VisionSubsystem(config);
        subsystems.add(visionSubsystem);

        if (config.enableShooterSubsystem) {
            shooterSubsystem = new ShooterSubsystem(config, visionSubsystem);
            subsystems.add(shooterSubsystem);
        }

        if (config.enableDriveSubsystem && config.enableShooterSubsystem) {
            navigationSubsystem = new NavigationSubsystem(config, visionSubsystem, shooterSubsystem);
            driveSubsystem = new DriveSubsystem(config, navigationSubsystem, visionSubsystem);
            navigationSubsystem.setDrive(driveSubsystem); // Java
            subsystems.add(driveSubsystem);
            subsystems.add(navigationSubsystem);
        }

        if (config.enableIntakeSubsystem) {
            intakeSubsystem = new IntakeSubsystem(config);
            subsystems.add(intakeSubsystem);
        }

        if (config.enableSpinnyboiSubsystem) {
            spinnyBoiSubsystem = new SpinnyBoiSubsystem(config);
            subsystems.add(spinnyBoiSubsystem);
        }

        if (config.enableClimbSubsystem) {
            climbSubsystem = new ClimbSubsystem(config);
            subsystems.add(climbSubsystem);
        }

        if (config.enablePIDHelper) {
            subsystems.add(new PIDHelperSubsystem(config));
        }

        wireUpButtons();

        //////////

        canChecker = new CANChecker();

        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.initialize();
            subsystem.listTalons();
            canChecker.addTalons(subsystem.getTalons());
        }

        // The first argument is the root container
        // The second argument is whether logging and config should be given separate tabs
        // NOTE: This must happen after robot initialization so the Logged fields are all valid objects
        Logger.configureLoggingAndConfig(this, false);
        Logger.configureLoggingAndConfig(config, false);

        lastTime = System.currentTimeMillis();
    }

    public void wireUpButtons() {

        Buttons b = new Buttons();

        //////////////////////////////////////////////
        // Drive Subsystem
        if (config.enableDriveSubsystem) {
            b.driveAutoAim.whenHeld(new AutoAlign(driveSubsystem));

            driveSubsystem.setDefaultCommand(new RunCommand(
                () -> driveSubsystem.drive(
                    b.driverControl.getRawAxis(b.driveSpeedAxis),
                    b.driverControl.getRawAxis(b.driveTurnAxis)),
                driveSubsystem)
            );
        }

        //////////////////////////////////////////////
        // Intake Subsystem
        if (config.enableIntakeSubsystem) {
            b.operatorToggleIntake.whenPressed(new InstantCommand(() -> intakeSubsystem.toggleIntakeArm(), intakeSubsystem));

            b.operatorIntake.whenHeld(new InstantCommand(() -> intakeSubsystem.intake(), intakeSubsystem))
                    .whenReleased(new InstantCommand(() -> intakeSubsystem.off(), intakeSubsystem));
            b.operatorOutake.whenHeld(new InstantCommand(() -> intakeSubsystem.outake(), intakeSubsystem))
                    .whenReleased(new InstantCommand(() -> intakeSubsystem.off(), intakeSubsystem));
        }

        //////////////////////////////////////////////
        // Climb Subsystem
        if (config.enableClimbSubsystem) {
            b.operatorActivateClimb.and(b.driveActivateClimb).whenActive(new InstantCommand(() -> climbSubsystem.toggleActive(), climbSubsystem));

            // continuously call moveArms() with the axis valuse
            // moveArms() will apprpriately call pitRewind or manualClimb
            climbSubsystem.setDefaultCommand(new RunCommand(
                () -> climbSubsystem.moveArms(
                    b.operatorControl.getRawAxis(b.operatorClimbLeft), 
                    b.operatorControl.getRawAxis(b.operatorClimbRight)), 
                climbSubsystem)
            );
            
        }


        //////////////////////////////////////////////
        // Shooter Subsystem
        if (config.enableShooterSubsystem) {
            b.operatorAimBot.whenHeld(new InstantCommand(() -> shooterSubsystem.autoAim(), shooterSubsystem))
                    .whenReleased(new InstantCommand(() -> shooterSubsystem.stopAutoAim(), shooterSubsystem));
            b.operatorSpinUp.whenHeld(new InstantCommand(() -> shooterSubsystem.startSpinningUp(), shooterSubsystem))
                    .whenReleased(new InstantCommand(() -> shooterSubsystem.stopSpinningUp(), shooterSubsystem));
            b.operatorFeeder.whenHeld(new InstantCommand(() -> shooterSubsystem.spinFeeder(), shooterSubsystem))
                    .whenReleased(new InstantCommand(() -> shooterSubsystem.stopSpinningFeeder(), shooterSubsystem));
            b.operatorFire.whenHeld(new InstantCommand(() -> shooterSubsystem.spinBMS(), shooterSubsystem))
                    .whenReleased(new InstantCommand(() -> shooterSubsystem.holdFire(), shooterSubsystem));

            shooterSubsystem.setDefaultCommand(new RunCommand(
                () -> shooterSubsystem.rotate(
                    Math.abs(b.operatorControl.getRawAxis(b.operatorAzimuth)), 
                    Math.abs(b.operatorControl.getRawAxis(b.operatorElevation))),
                shooterSubsystem)
            );

            b.operatorNextElevation.whenPressed(new StartEndCommand(
                () -> shooterSubsystem.nextPositionElevation(),
                () -> shooterSubsystem.resetPositionElevationSwitcher(),
                shooterSubsystem)
            );
            
            b.operatorLastElevation.whenPressed(new StartEndCommand(
                () -> shooterSubsystem.lastPositionElevation(),
                () -> shooterSubsystem.resetPositionElevationSwitcher(),
                shooterSubsystem)
            );

            b.setElevationToDashboardNum.whenPressed(new InstantCommand(() -> {
                shooterSubsystem.rotateToDeg(
                    shooterSubsystem.getTargetAzimuthDeg(),
                    SmartDashboard.getNumber(shooterSubsystem.getName() + "/Dashboard Elevation Target", config.shooter.DEFAULT_ELEVATION_TARGET_DEG)
                );
            }, shooterSubsystem));


            b.operatorZero.whenPressed(new InstantCommand(() -> shooterSubsystem.rotateToDeg(0, 0), shooterSubsystem));

        }

        //////////////////////////////////////////////
        // SpinnyBoi Subsystem
        if (config.enableSpinnyboiSubsystem) {
            b.operatorSpinForward.whenHeld(new InstantCommand(() -> spinnyBoiSubsystem.forward(), spinnyBoiSubsystem)).whenReleased(new InstantCommand(() -> spinnyBoiSubsystem.off(), spinnyBoiSubsystem));
            b.operatorSpinBackward.whenHeld(new InstantCommand(() -> spinnyBoiSubsystem.backward(), spinnyBoiSubsystem)).whenReleased(new InstantCommand(() -> spinnyBoiSubsystem.off(), spinnyBoiSubsystem));
        }
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
        canChecker.periodic();

        currentTime = System.currentTimeMillis();
        deltaTime = (currentTime - lastTime) / 1000f;
        SmartDashboard.putNumber("deltaTime", deltaTime);

        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.periodic(deltaTime);
            subsystem.dashboardPeriodic(deltaTime);
        }

        CommandScheduler.getInstance().run();

        // update Oblog entries
        Logger.updateEntries();

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
        // reset nav system, turn on LL leds
        navigationSubsystem.reset();
        navigationSubsystem.setInitialPose(driveSubsystem.getFirstPickupTrajectory().getInitialPose());
        visionSubsystem.turnOnLEDs();
        // begin spinning up our hopper
        shooterSubsystem.spinBMS();

        //intakeSubsystem.toggleIntakeArm();

        // start the command sequence
        // and turn the intake
        new InstantCommand(() -> {
            intakeSubsystem.intake();
        })

        // do the first pickup and return
        .andThen(new AutoDrive(driveSubsystem, driveSubsystem.getFirstPickupTrajectory()))
        .andThen(new AutoDrive(driveSubsystem, driveSubsystem.getFirstReturnTrajectory()))

        // raise the hood angle and sh00t
        .andThen(() -> shootStuff())

        // wait until we're done shooting
        .andThen(new WaitUntilCommand(() -> {
            System.out.println("Wait until");
            return shooterSubsystem.isUpToSpeed();
        }))
        // the isUpToSpeed() was broken or something so we added this for a fix? dont quite remember
        // either way, we wait to make *sure* we're done shooting
        .andThen(new WaitCommand(3))

        // stop shooting and lower the hood
        .andThen(new InstantCommand(() -> {
            shooterSubsystem.stopSpinningUp();
            shooterSubsystem.stopSpinningFeeder();
            shooterSubsystem.rotateToDeg(0, 0);
        }))

        // so that we're sure that the hood angle can go down
        .andThen(new WaitCommand(1))

        // only execute the 2nd pickup and return if we have the trajectories for those
        // if we have a pickup, we'll have a return; so we don't have to worry about that
        .andThen(new ConditionalCommand(
            new ProxyCommand(() ->

                // get the second set of balls and shoot
                new AutoDrive(driveSubsystem, driveSubsystem.getSecondPickupTrajectory())
                .andThen(new AutoDrive(driveSubsystem, driveSubsystem.getSecondReturnTrajectory()))
                .andThen(() -> shootStuff())

                // to shoot the shots
                .andThen(new WaitUntilCommand(() -> {
                    System.out.println("Wait until");
                    return shooterSubsystem.isUpToSpeed();
                }))
                .andThen(new WaitCommand(3))

                // we are done so stop it
                .andThen(() -> stopEverything())
            ),
            // stop it if we dont have a second path
            new InstantCommand(() -> {
                stopEverything();
            }),
            // only IF we have a second trajectory in the first place
            driveSubsystem::hasSecondTrajectory
        ))

        // stop everything
        .andThen(() -> stopEverything())

        // all done
        .schedule();
    }

    /**
     * helper method to disable everything we need once we're done
     */
    public void stopEverything(){
        driveSubsystem.tankVolts(0, 0);
        shooterSubsystem.stopSpinningFeeder();
        shooterSubsystem.stopSpinningUp();
        shooterSubsystem.holdFire();
        shooterSubsystem.rotateToDeg(0, 0);
        intakeSubsystem.off();
    }

    /**
     * helper method to do everything we need to shoot a ball
     */
    public void shootStuff() {
        // stop moving
        driveSubsystem.tankVolts(0, 0);
        // rotate the hood angle to 45deg
        shooterSubsystem.rotateToDeg(0, 45);
        // start spinning up the shooter
        shooterSubsystem.startSpinningUp();
        // spin the feeder (which feeds to the shooter)
        shooterSubsystem.spinFeeder();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        visionSubsystem.turnOnLEDs();

        if (config.enableShooterSubsystem) {
            shooterSubsystem.rotateToDeg(0, config.shooter.DEFAULT_ELEVATION_TARGET_DEG);
        }

        if (config.enableClimbSubsystem) {
            climbSubsystem.disableClimb();
            climbSubsystem.disableRewind();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        //////////////////////////////////////////////////////////////////////////////
        // Shooter Subsystem
        if (config.enableShooterSubsystem) {
            SmartDashboard.putNumber("BallManagementSubsystem/Output Percent", 50);            
        }
    }

    @Override
    public void testInit() {
        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.testInit();
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.testPeriodic();
        }
    }

    @Override
    public void disabledInit() {
        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.disable();
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    // COMMANDS the robot to WIN!
    public static Robot win() {
        System.out.println("Leif WAS here");

        return new Robot();
    }

    public static Robot beat(int teamNumber) {
        return win();
    }

    public static Robot beat254() {
        return beat(254);
    }
}