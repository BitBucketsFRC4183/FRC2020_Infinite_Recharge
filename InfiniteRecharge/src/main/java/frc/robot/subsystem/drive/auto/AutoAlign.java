package frc.robot.subsystem.drive.auto;

import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.drive.Idle;
import frc.robot.subsystem.drive.RotationDrive;
import frc.robot.subsystem.drive.VelocityDrive;
import frc.robot.subsystem.drive.DriveSubsystem.DriveMethod;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
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
    private final VisionSubsystem VISION_SUBSYSTEM;
    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;



    public AutoAlign(DriveSubsystem driveSubsystem) {
        DRIVE_SUBSYSTEM = driveSubsystem;
        VISION_SUBSYSTEM = DRIVE_SUBSYSTEM.getVision();
        NAVIGATION_SUBSYSTEM = DRIVE_SUBSYSTEM.getNavigation();
    }

    public void initialize() {
        
    }

    @Override
    public void execute() {
        // Navigation yaw is defined as + for CCW but tx is defined as - for when it should turn CCW
        double tx = VISION_SUBSYSTEM.getFilteredTx(-NAVIGATION_SUBSYSTEM.getYaw_deg());

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