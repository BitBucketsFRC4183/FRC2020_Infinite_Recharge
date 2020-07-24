package frc.robot.subsystem.drive.auto;

import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


// now we just align the turret so we don't use this
public class AutoAlign extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final VisionSubsystem VISION_SUBSYSTEM;
    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;

    private final PIDController PID;



    public AutoAlign(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
        VISION_SUBSYSTEM = DRIVE_SUBSYSTEM.getVision();
        NAVIGATION_SUBSYSTEM = DRIVE_SUBSYSTEM.getNavigation();

        // PID controller to convert angle error to angular velocity
        PID = new PIDController(
            DriveConstants.AUTO_ALIGN_KP,
            DriveConstants.AUTO_ALIGN_KI,
            DriveConstants.AUTO_ALIGN_KD
        );
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        // Navigation yaw is defined as + for CCW but tx is defined as - for when it should turn CCW
        double tx = VISION_SUBSYSTEM.getFilteredTx(-NAVIGATION_SUBSYSTEM.getYaw_deg());

        double omega_radps = PID.calculate(-tx);
        //DRIVE_SUBSYSTEM.velocityDrive_auto(0, omega_radps);
    }
}