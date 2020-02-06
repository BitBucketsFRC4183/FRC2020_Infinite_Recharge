package frc.robot.subsystem.drive;

import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.utils.CommandUtils;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



public class AutoDrive extends RamseteCommand {
    private final DriveSubsystem DRIVE_SUBSYSTEM;



    public AutoDrive(DriveSubsystem driveSubsystem) {
        super(
            driveSubsystem.getAutoTrajectory(),
            driveSubsystem::getPose,
            new RamseteController(),
            driveSubsystem.getCharacterization(),
            driveSubsystem.getKinematics(),
            null,//driveSubsystem::getWheelSpeeds(),
            driveSubsystem.getLeftPID(),
            driveSubsystem.getLeftPID(),
            null,//driveSubsystem::tankVolts(),
            driveSubsystem
        );

        DRIVE_SUBSYSTEM = driveSubsystem;
    }



    public void initialize() {
        DRIVE_SUBSYSTEM.disable();
    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.ROTATION) {
            return CommandUtils.stateChange(new RotationDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}