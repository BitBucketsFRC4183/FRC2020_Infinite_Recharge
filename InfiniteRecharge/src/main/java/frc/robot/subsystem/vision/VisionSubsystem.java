package frc.robot.subsystem.vision;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.math.MathUtils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionSubsystem extends BitBucketSubsystem {

    private boolean validTarget = false;
    
    private final double defaultVal = 0.0;
    private NetworkTable limelightTable;

    private double tx = 0;
    private double ty = 0;

    private double distance = 0;
    private double zoom = 1;
    private double pan = 0;

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
        distance = approximateDistanceFromTarget(ty);
        adjustZoom();

        SmartDashboard.putBoolean(getName() + "/Valid Target ", validTarget);
        SmartDashboard.putNumber(getName() + "/Estimated Distance ", distance);
    }

    public double approximateDistanceFromTarget(final double ty) {
        double distance_no_zoom = (VisionConstants.TARGET_HEIGHT_INCHES - VisionConstants.CAMERA_HEIGHT_INCHES)
                / Math.tan(Math.toRadians(VisionConstants.CAMERA_MOUNTING_ANGLE + ty));
        return distance_no_zoom * zoom;
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
        ty -= pan;
    }

    public void adjustZoom() {
        double pipelineToChangeTo = 0;

        //TODO: empirically test this
        //higher zoom (higher pipeline) the further u go
        if (distance >= 0) {
            pipelineToChangeTo = 0;
        }
        if (distance >= 200) {
            pipelineToChangeTo = 1;
        }
        if (distance >= 400) {
            pipelineToChangeTo = 2;
        }

        // while (!validTarget) {
        //     if (pipelineToChangeTo == 2) {
        //         pipelineToChangeTo = 0;
        //     }
        //     pipelineToChangeTo++;
        // }

        // System.out.println(validTarget);
        SmartDashboard.putNumber(getName() + "/Pipeline to Change to", pipelineToChangeTo);
        // System.out.println(pipelineToChangeTo);

        zoom = pipelineToChangeTo + 1;
        // SmartDashboard.getNumber(getName() + "/Pipeline to Change to", efaultValue)
        limelightTable.getEntry("pipeline").setDouble(pipelineToChangeTo);

        if (pipelineToChangeTo != 0)
            pan = 1;
    }

    public void turnOnLEDs() {
        limelightTable.getEntry("ledMode").setNumber(3);
    }

    public void turnOffLEDs() {
        limelightTable.getEntry("ledMode").setNumber(1);
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public boolean getValidTarget() {
        return validTarget;
    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
    }

    @Override
    protected void listTalons() {}
}