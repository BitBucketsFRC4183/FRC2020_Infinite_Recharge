package frc.robot.subsystem.drive;



import frc.robot.operatorinterface.OI;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class RotationDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final OI OI;



    public RotationDrive(DriveSubsystem driveSubsystem, OI oi) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
        OI = oi;
    }



    public void execute() {

    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() != DriveSubsystem.DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.AUTO) {
            return CommandUtils.stateChange(new AutoDrive(DRIVE_SUBSYSTEM, OI));
        }

        return false;
    }
}