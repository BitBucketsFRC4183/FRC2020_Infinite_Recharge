package frc.robot.subsystem.drive;



import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.utils.CommandUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutoDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final OI OI;



    public AutoDrive(DriveSubsystem driveSubsystem, OI oi) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
        OI = oi;
    }



    public void initialize() {
        DRIVE_SUBSYSTEM.disable();
    }



    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.ROTATION) {
            return CommandUtils.stateChange(new RotationDrive(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM, OI));
        }

        return false;
    }
}