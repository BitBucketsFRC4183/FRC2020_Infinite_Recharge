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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.spinnyboi.SpinnyBoiSubsystem;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.climber.ClimbSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.ProxyCommand;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.drive.DriveUtils;
import frc.robot.subsystem.drive.Idle;
import frc.robot.subsystem.drive.auto.AutoDrive;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.pidhelper.PIDHelperSubsystem;
import frc.robot.subsystem.scoring.intake.IntakeSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterConstants;
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

    private final OI oi = new OI();

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

        canChecker = new CANChecker();

        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.initialize();
            subsystem.listTalons();
            canChecker.addTalons(subsystem.getTalons());
        }

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
        canChecker.periodic();

        currentTime = System.currentTimeMillis();
        deltaTime = (currentTime - lastTime) / 1000f;
        SmartDashboard.putNumber("deltaTime", deltaTime);

        for (BitBucketSubsystem subsystem : subsystems) {
            subsystem.periodic(deltaTime);
            subsystem.dashboardPeriodic(deltaTime);
        }

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
        .andThen(new Idle(driveSubsystem))
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
        new Idle(driveSubsystem);
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
            shooterSubsystem.rotateToDeg(0, ShooterConstants.DEFAULT_ELEVATION_TARGET_DEG);
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
        // Drive Subsystem

        if (config.enableDriveSubsystem) {
            driveSubsystem.setDriverRawSpeed(oi.speed());
            driveSubsystem.setDriverRawTurn(oi.turn());

            driveSubsystem.setAutoAligning(oi.driveAimBot());
        }

        //////////////////////////////////////////////////////////////////////////////
        // Intake Subsystem

        if (config.enableIntakeSubsystem) {
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
        }
        /////////////////////////////////////////////////////////////////////////////
        // Climb Subsystem
        if (config.enableClimbSubsystem) {
            if (oi.climbActivate()) {
                climbSubsystem.toggleActive();
            }

            if (climbSubsystem.isActive()) {
                if (!climbSubsystem.isRewindEnabled()) {
                    climbSubsystem.manualClimb(oi.manualClimbLeft(),oi.manualClimbRight());
                } else if (climbSubsystem.isRewindEnabled()) {
                    climbSubsystem.pitRewind(oi.pitRewindLeft(), oi.pitRewindRight());
                }
            }

        }
        //////////////////////////////////////////////////////////////////////////////
        // Shooter Subsystem

        if (config.enableShooterSubsystem) {
            SmartDashboard.putNumber("BallManagementSubsystem/Output Percent", 50);

            if (oi.feeder()){
                shooterSubsystem.spinFeeder();
            } else {
                shooterSubsystem.stopSpinningFeeder();
            }

            // Spin up on pressing [spinUp] and auto aim on pressing [aimBot]
            if (oi.spinUp()) {
                shooterSubsystem.startSpinningUp();
            } else if (oi.operatorAimBot()) {
                shooterSubsystem.autoAim();
            } else {
                shooterSubsystem.stopSpinningUp();
                shooterSubsystem.stopAutoAim();
            }

            // Fire on pressing [fire]
            if (oi.fire()) {
                shooterSubsystem.spinBMS();
            } else {
                shooterSubsystem.holdFire();
            }

            // Rotate the turret with [manualAzimuthAxis]
            if (Math.abs(oi.manualAzimuthAxis()) >= config.shooter.manualAzimuthDeadband
                    || Math.abs(oi.manualElevationAxis()) >= config.shooter.manualElevationDeadband) {
                shooterSubsystem.rotate(oi.manualAzimuthAxis(), oi.manualElevationAxis());
            } else {
                shooterSubsystem.rotate(0, 0);
            }

            if (oi.zero()) {
                shooterSubsystem.rotateToDeg(0, 0);
            }

            if (oi.nextPositionElevation()) {
                shooterSubsystem.nextPositionElevation();
            } else if (oi.lastPositionElevation()) {
                shooterSubsystem.lastPositionElevation();
            } else {
                shooterSubsystem.resetPositionElevationSwitcher();
            }

            if (oi.setElevationToDashboardNumber()) {
                shooterSubsystem.rotateToDeg(shooterSubsystem.getTargetAzimuthDeg(),
                        SmartDashboard.getNumber(shooterSubsystem.getName() + "/Dashboard Elevation Target", ShooterConstants.DEFAULT_ELEVATION_TARGET_DEG));
            }
        }

        // //////////////////////////////////////////////////////////////////////////////
        // // SpinnyBoi Subsystem
        if (config.enableSpinnyboiSubsystem) {
            if (oi.spinnyBoiForward()) {
                spinnyBoiSubsystem.forward();
            } else if (oi.spinnyBoiBackward()) {
                spinnyBoiSubsystem.backward();
            } else {
                spinnyBoiSubsystem.off();
            }
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