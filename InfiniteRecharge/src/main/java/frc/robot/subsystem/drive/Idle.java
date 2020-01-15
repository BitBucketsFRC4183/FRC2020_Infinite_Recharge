package frc.robot.subsystem.drive;



import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.utils.CommandUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class Idle extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;



    public Idle(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
    }



    public void initialize() {
        DRIVE_SUBSYSTEM.disable();
    }



    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.ROTATION) {
            return CommandUtils.stateChange(new RotationDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.AUTO) {
            return CommandUtils.stateChange(new AutoDrive(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}