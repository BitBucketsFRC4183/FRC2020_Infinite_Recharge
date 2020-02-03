package frc.robot.subsystem.vision;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystem.scoring.shooter.ShooterConstants;
import frc.robot.utils.math.MathUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends BitBucketSubsystem {

    private double degreesToRotate = 0.0;
    private boolean validTarget = false;    

    private double defaultVal = 0;
    NetworkTable limelightTable;

    public double tx = 0;

    public VisionSubsystem(Config config) {
        super(config);
    }

    public void initialize() {
        super.initialize();

        NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
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
    public void periodic(float deltaTime) {

        updateTargetInfo();

        SmartDashboard.putBoolean(getName() + "/Valid Target ", validTarget);
    }

    public double getShooterVelocityForTarget() {
        double ty = queryLimelightNetworkTable("ty");
        
        double d = approximateDistanceFromTarget(ty);
        double h = ShooterConstants.TARGET_HEIGHT_INCHES;
        double angle = ShooterConstants.BALL_SHOOTING_ANGLE;

        double numerator = MathUtils.G * Math.pow(d, 2);
        double denominator = 2 * (d * Math.tan(angle) - h) * Math.pow(Math.cos(angle), 2);

        double vel = Math.sqrt(numerator / denominator);

        return vel;
    }

    public double approximateDistanceFromTarget(double ty) {
        return (ShooterConstants.TARGET_HEIGHT_INCHES - ShooterConstants.CAMERA_HEIGHT_INCHES) / Math.tan(ShooterConstants.CAMERA_MOUNTING_ANGLE + ty);
    }

    public double queryLimelightNetworkTable(String value) {
        return limelightTable.getEntry(value).getDouble(defaultVal);
    }

	public void updateTargetInfo() {
        
        double tv = queryLimelightNetworkTable("tv");
        if (tv == 1) {
            validTarget = true;
        } else {
            validTarget = false;
        }

        tx = queryLimelightNetworkTable("tx");
    }

    public double getDegreesToRotate() {
        return degreesToRotate;
    }

    public double getTx() {
        return tx;
    }

}