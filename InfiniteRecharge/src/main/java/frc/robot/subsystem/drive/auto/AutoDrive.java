package frc.robot.subsystem.drive.auto;

import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.drive.Idle;
import frc.robot.subsystem.drive.RotationDrive;
import frc.robot.subsystem.drive.VelocityDrive;
import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.utils.CommandUtils;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



public class AutoDrive extends RamseteCommand {
    private final DriveSubsystem DRIVE_SUBSYSTEM;



    public AutoDrive(DriveSubsystem driveSubsystem) {
        super(
            driveSubsystem.getAutoTrajectory(),
            driveSubsystem::getPose,
            new RamseteController(),
            driveSubsystem.getKinematics(),
            driveSubsystem::setWheelSpeeds,
            driveSubsystem
        );

        DRIVE_SUBSYSTEM = driveSubsystem;
    }



    public void initialize() {
        DRIVE_SUBSYSTEM.disable();
    }



    @Override
    public boolean isFinished() {
        // whether the RAMSETE command trajectory has been executed
        boolean trajDone = super.isFinished();

        // go into idle once trajectory has been finished
        if (trajDone) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}