package frc.robot.subsystem.drive;

import frc.robot.subsystem.drive.auto.AutoAlign;
import frc.robot.subsystem.drive.auto.AutoDrive;
import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class VelocityDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;



    public VelocityDrive(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
    }



    public void execute() {
        double rawSpeed = DRIVE_SUBSYSTEM.getDriverRawSpeed();
        double rawTurn = DRIVE_SUBSYSTEM.getDriverRawTurn();

        DRIVE_SUBSYSTEM.velocityDrive(rawSpeed, rawTurn);

        

        // limit acceleration if needed
        //speed_ips = SLEW_FILTER.calculate(speed_ips);
    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.ALIGN) {
            return CommandUtils.stateChange(new AutoAlign(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}