package frc.robot.subsystem.vision;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.data.filters.RunningAverageFilter;
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

    RunningAverageFilter txFilter = new RunningAverageFilter(VisionConstants.FILTER_LENGTH);

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
        autoZoom();

        SmartDashboard.putNumber(getName() + "/Zoom", zoom);
        SmartDashboard.putNumber(getName() + "/Pan", pan);
        SmartDashboard.putBoolean(getName() + "/Valid Target ", validTarget);
        SmartDashboard.putNumber(getName() + "/Estimated Distance ", distance);
    }

    /**
     * Uses the formula found https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     * d = (h2-h1) / tan(a1+a2)
     * h2 -> height of target
     * h1 -> height of cmera
     * a1 -> mounting angle of the camera
     * a2 -> ty
     * @param ty The vertical offset from the target, given by the LL
     * @return The estimated distance from the target
     */
    public double approximateDistanceFromTarget(final double ty) {
        double distance_no_zoom = (VisionConstants.TARGET_HEIGHT_INCHES - VisionConstants.CAMERA_HEIGHT_INCHES)
                / Math.tan(Math.toRadians(VisionConstants.CAMERA_MOUNTING_ANGLE + ty));
        // LL doesn't account for zoom so we have to do that as well
        // Zoom is updated periodically in autoZoom()
        return distance_no_zoom * zoom;
    }

    /**
     * [NOT USED ANYWHERE] Finds ty from distance. Only implemented for testing for localization
     * The formula is just the distance one but to solve for ty
     * @param distance Estimated distance from target in inches
     * @return Approximate value of ty
     */
    public double approximateTyFromDistance(final double distance) {
        final double ty = Math.toDegrees(
                Math.atan((VisionConstants.TARGET_HEIGHT_INCHES - VisionConstants.CAMERA_HEIGHT_INCHES) / distance))
                - VisionConstants.CAMERA_MOUNTING_ANGLE;

        return ty;
    }

    /**
     * Helper method to get a value from the LL NT
     * @param value key you want to get
     * @return value associated with the key passed in
     */
    public double queryLimelightNetworkTable(final String value) {
        return limelightTable.getEntry(value).getDouble(defaultVal);
    }

    /**
     * Update tv (valid target) and tx and ty (horizontal and vertical offsets from
     * target)
     */
    public void updateTargetInfo() {

        final double tv = queryLimelightNetworkTable("tv");
        if (tv == 1) {
            validTarget = true;

            // Add the tx offset (for drive auto-align) to the tx
            tx = queryLimelightNetworkTable("tx") + VisionConstants.TX_BIAS_DEG;
            ty = queryLimelightNetworkTable("ty");
            // LL doesn't count for the pan so we have to that ourselves
            ty -= pan;
        } else {
            validTarget = false;
        }
    }

    /**
     * Changes to the correct LL pipeline (which are correspondingly set to zoom levels) based on the distance from the target
     * We never really got a lot of time to test this sooooooo
     */
    public void autoZoom() {
        // Like we said, we never got a lot of time to test it
        // So sometimes we wanted to test the robot but not mess with vision
        // In that case, we disable auto zoom (and change the pipelines manually)
        // But we still have to set the zoom and pan, which is why this if block exists
        if (!VisionConstants.ENABLE_AUTO_ZOOM) {
            // Get the current pipeline
            double currentPipeline = limelightTable.getEntry("getpipe").getDouble(0);
            // In our case in happened to be that the zoom was just the pipeline number plus 1
            zoom = currentPipeline + 1;
            // The pan was always set to 1 except for pipeline 0 (1x zoom)
            // Therefore this
            if (currentPipeline != 0)
                pan = 1;
            // We dont return anything
            return;
        }

        double pipelineToChangeTo = 0;

        // TODO: empirically test this
        // higher zoom (higher pipeline) the further u go
        if (distance >= 0) {
            pipelineToChangeTo = 0;
        }
        if (distance >= 200) {
            pipelineToChangeTo = 1;
        }
        if (distance >= 400) {
            pipelineToChangeTo = 2;
        }

        // attempted to cycle through pipline if no valid target
        // but it locked up the whole code and we never got time to fix it and put it on a different thread or something

        // while (!validTarget) {
        // if (pipelineToChangeTo == 2) {
        // pipelineToChangeTo = 0;
        // }
        // pipelineToChangeTo++;
        // }

        SmartDashboard.putNumber(getName() + "/Pipeline to Change to", pipelineToChangeTo);

        zoom = pipelineToChangeTo + 1;
        // SmartDashboard.getNumber(getName() + "/Pipeline to Change to", 0)
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

    /** offset adds to tx, where offset + tx must be a constant */
    public double getFilteredTx(double offset) {
        if (VisionConstants.USE_FILTER) {
            // make sure there's a valid target before adding new value
            if (validTarget) {
                // calculate the running average of tx and the offset then remove the offset to not mess with the result
                return txFilter.calculate(tx + offset) - offset;
            } else {
                // use last average if no target but it's enabled
                return txFilter.getAverage() - offset;
            }
        } else {
            // just return raw value if no filter
            return tx;
        }
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

    public void disable() {
        turnOffLEDs();
    }

    @Override
    public void listTalons() {
    }
}