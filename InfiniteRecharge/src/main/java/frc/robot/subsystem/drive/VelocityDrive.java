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

        // Scale the input to physical units (with implied limits)
		double speed_ips = MathUtils.map(rawSpeed,
            -1.0,
            1.0,
            -DriveConstants.MAX_ALLOWED_SPEED_IPS,
            DriveConstants.MAX_ALLOWED_SPEED_IPS
        );

        double turn_radps = MathUtils.map(rawTurn,
            -1.0,
            1.0,
            -DriveConstants.MAX_ALLOWED_TURN_RADPS,
            DriveConstants.MAX_ALLOWED_TURN_RADPS
        );

        // limit acceleration if needed
        speed_ips = SLEW_FILTER.calculate(speed_ips);



        DRIVE_SUBSYSTEM.velocityDrive(speed_ips, turn_radps);
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