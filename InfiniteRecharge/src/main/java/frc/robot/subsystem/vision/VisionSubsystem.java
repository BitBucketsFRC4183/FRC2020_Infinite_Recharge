package frc.robot.subsystem.vision;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.math.MathUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends BitBucketSubsystem {

    private boolean validTarget = false;
    
    private final double defaultVal = 0.0;
    private NetworkTable limelightTable;

    private double tx = 0;
    private double ty = 0;

    public VisionSubsystem(final Config config) {
        super(config);
    }

    public void initialize() {
        super.initialize();

        final NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
        tableInstance.startClientTeam(4183);

        limelightTable = tableInstance.getTable("limelight");
    }

    public void diagnosticsInitialize() {

    }

    public void diagnosticsPeriodic() {

    }

    public void diagnosticsCheck() {

    }

    @Override
    public void periodic(final float deltaTime) {

        updateTargetInfo();
        final double distance = approximateDistanceFromTarget(ty);

        SmartDashboard.putBoolean(getName() + "/Valid Target ", validTarget);
        SmartDashboard.putNumber(getName() + "/Estimated Distance ", distance);
    }

    public double getShooterVelocityForTarget() {

        final double d = approximateDistanceFromTarget(ty);
        final double h = VisionConstants.TARGET_HEIGHT_INCHES;
        final double angle = VisionConstants.BALL_SHOOTING_ANGLE;

        final double numerator = MathUtils.G * Math.pow(d, 2);
        final double denominator = 2 * (d * Math.tan(angle) - h) * Math.pow(Math.cos(angle), 2);

        final double vel = Math.sqrt(numerator / denominator);

        return vel;
    }

    public double approximateDistanceFromTarget(final double ty) {
        return (VisionConstants.TARGET_HEIGHT_INCHES - VisionConstants.CAMERA_HEIGHT_INCHES)
                / Math.tan(VisionConstants.CAMERA_MOUNTING_ANGLE + ty);
    }

    public double queryLimelightNetworkTable(final String value) {
        return limelightTable.getEntry(value).getDouble(defaultVal);
    }

	public void updateTargetInfo() {
        
        final double tv = queryLimelightNetworkTable("tv");
        if (tv == 1) {
            validTarget = true;
        } else {
            validTarget = false;
        }

        tx = queryLimelightNetworkTable("tx");
        ty = queryLimelightNetworkTable("ty");
    }

    public double getTx() {
        return tx;
    }

    public boolean getValidTarget() {
        return validTarget;
    }

}