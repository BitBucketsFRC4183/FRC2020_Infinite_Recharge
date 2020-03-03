package frc.robot.subsystem.drive.auto;

import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.drive.Idle;
import frc.robot.subsystem.drive.RotationDrive;
import frc.robot.subsystem.drive.VelocityDrive;
import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;
import frc.robot.utils.CommandUtils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;



public class AutoAlign extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final ShooterSubsystem SHOOTER_SUBSYSTEM;



    public AutoAlign(DriveSubsystem driveSubsystem) {
        DRIVE_SUBSYSTEM = driveSubsystem;
        SHOOTER_SUBSYSTEM = DRIVE_SUBSYSTEM.getShooter();
    }

    public void initialize() {
        
    }

    @Override
    public void execute() {
        double tx = SHOOTER_SUBSYSTEM.getDegreesToRotate();

        DRIVE_SUBSYSTEM.velocityDrive_auto(0, -tx * DriveConstants.AUTO_ALIGN_KP);
    }

    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM));
        }

        return false;
    }
}