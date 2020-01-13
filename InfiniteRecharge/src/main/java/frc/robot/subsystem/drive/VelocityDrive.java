package frc.robot.subsystem.drive;



import frc.robot.operatorinterface.OI;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class VelocityDrive extends CommandBase {
    private final DriveSubsystem DRIVE_SUBSYSTEM;
    private final OI OI;



    private final SlewRateLimiter SLEW_FILTER;

    public VelocityDrive(DriveSubsystem driveSubsystem, OI oi) {
        addRequirements(driveSubsystem);

        DRIVE_SUBSYSTEM = driveSubsystem;
        OI = oi;

        double vel_ips = DRIVE_SUBSYSTEM.getSpeed_ips();
        SLEW_FILTER = new SlewRateLimiter(DriveConstants.MAX_LIN_ACCELERATION_IPSPS, vel_ips);
    }



    public void execute() {
        double rawSpeed = OI.speed();
        double rawTurn = OI.turn();

        DRIVE_SUBSYSTEM.velocityDrive(rawSpeed, rawTurn);

        

        // limit acceleration if needed
        //speed_ips = SLEW_FILTER.calculate(speed_ips);



        DRIVE_SUBSYSTEM.velocityDrive(rawSpeed, rawTurn);
    }



    @Override
    public boolean isFinished() {
        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.ROTATION) {
            return CommandUtils.stateChange(new RotationDrive(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.AUTO) {
            return CommandUtils.stateChange(new AutoDrive(DRIVE_SUBSYSTEM, OI));
        }

        if (DRIVE_SUBSYSTEM.getDriveMethod() == DriveSubsystem.DriveMethod.IDLE) {
            return CommandUtils.stateChange(new Idle(DRIVE_SUBSYSTEM, OI));
        }

        return false;
    }
}