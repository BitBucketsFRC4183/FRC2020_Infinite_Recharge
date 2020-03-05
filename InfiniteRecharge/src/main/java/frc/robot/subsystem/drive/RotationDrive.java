package frc.robot.subsystem.drive;

import frc.robot.subsystem.drive.auto.AutoAlign;
import frc.robot.subsystem.drive.auto.AutoDrive;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class RotationDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;

    private double yaw0;



    public RotationDrive(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
        NAVIGATION_SUBSYSTEM = DRIVE_SUBSYSTEM.getNavigation();

        yaw0 = NAVIGATION_SUBSYSTEM.getYaw_deg();
    }



    public void execute() {
        double rawSpeed = DRIVE_SUBSYSTEM.getDriverRawSpeed();
        double rawTurn = DRIVE_SUBSYSTEM.getDriverRawTurn();

        DRIVE_SUBSYSTEM.rotationDrive(rawSpeed, rawTurn, yaw0);
    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.VELOCITY) {
            return CommandUtils.stateChange(new VelocityDrive(DRIVE_SUBSYSTEM));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.ALIGN) {
            return CommandUtils.stateChange(new AutoAlign(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}