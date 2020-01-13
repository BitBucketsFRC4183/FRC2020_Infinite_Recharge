package frc.robot.subsystem.drive;



import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class RotationDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;



    public RotationDrive(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
    }



    public void execute() {

    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() != DriveSubsystem.DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.AUTO) {
            return CommandUtils.stateChange(new AutoDrive(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}