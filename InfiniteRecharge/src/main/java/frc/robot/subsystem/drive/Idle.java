package frc.robot.subsystem.drive;

import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.subsystem.drive.auto.AutoAlign;
import frc.robot.subsystem.drive.auto.AutoDrive;
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

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.ALIGN) {
            return CommandUtils.stateChange(new AutoAlign(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}