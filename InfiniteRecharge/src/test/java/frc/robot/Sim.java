package frc.robot;

import java.io.IOException;
import java.awt.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.simulator.SimMain;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs.MotorOutput;
import frc.robot.simulator.sim.RobotPosition;
import frc.robot.simulator.sim.SimulatorSettings;
import frc.robot.simulator.sim.RobotPosition.Type;
import frc.robot.simulator.sim.config.SimulatorConfig;
import frc.robot.simulator.sim.config.serialization.ConfigReader;
import frc.robot.simulator.sim.events.EventManager;
import frc.robot.simulator.sim.events.FieldRenderEvent;
import frc.robot.simulator.sim.events.RobotInitializedEvent;
import frc.robot.simulator.sim.field.wheeldisplacement.Field;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.vision.VisionConstants;
import frc.robot.utils.math.MathUtils;

public class Sim {

    // the coordinates of the blue target, in meters, on the field
    // this is based on the 0, 0 point being the center of the field image.
    private double targetX = 1.42;
    private double targetY = 8.35;
    private double aziumthMotorPositionInRadians = 0;

    // the angle the limelight can see a target, 60 degrees in either direction is probably too much, but...
    private static final double LIMELIGHT_VIEWING_ANGLE = 60 / (Math.PI / 2);

    public static void main(String[] args) throws InterruptedException, IOException {
        // create a new BitBuckets sim object to respond to simulator events
        Sim bitbucketsSim = new Sim();

        // subscribe to events from the simulator
        EventManager.subscribeToRobotInitializedEvents(bitbucketsSim::onRobotInitialized);
        EventManager.subscribeToMotorOutputsEvents(bitbucketsSim::onMotorOutputsUpdated);
        EventManager.subscribeToRobotPositionEvents(bitbucketsSim::onRobotPositionUpdated);
        EventManager.subscribeToFieldRenderEvents(bitbucketsSim::onFieldRender);

        // start the sim
        SimMain.main(args);
    }

    private final Config config;
    private boolean robotInitialized;
    private NetworkTable limelightTable;
    private SimulatorConfig simulatorConfig;
    private ConfigReader configReader = new ConfigReader(new SimulatorSettings());

    /**
     * Initialize the sim with a config for our robot, so we know what motor ids to
     * use
     */
    public Sim() {
        config = ConfigChooser.getConfig();
        try {
            simulatorConfig = configReader.loadConfig();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            simulatorConfig = new SimulatorConfig();
        }
    }

    /**
     * This is called when the robotInit() is complete, so we know we can start
     * checking motor outputs and configs.
     * 
     * @param robotInitializedEvent
     */
    void onRobotInitialized(RobotInitializedEvent robotInitializedEvent) {
        robotInitialized = true;

        NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
        tableInstance.startClientTeam(4183);

        limelightTable = tableInstance.getTable("limelight");
    }

    void onRobotPositionUpdated(RobotPosition robotPosition) {
        if (robotPosition.type == Type.WheelDisplacement) {
            double yDistance = targetY - robotPosition.y;
            double xOffset = targetX - robotPosition.x;

            // compute the angle (to the right) that the target is from our robot center
            double txRads;
            if (xOffset > 0) {
                txRads = (Math.PI / 2) - Math.atan(yDistance / xOffset);
            } else if (xOffset < 0) {
                txRads = -1 * ((Math.PI / 2) + Math.atan(yDistance / xOffset));
            } else {
                txRads = 0;
            }

            // if our robot is rotated to the right, update the tx by that offset
            txRads = txRads - robotPosition.heading;

            // if our turrent is rotated, update txRads by the azimuth change
            txRads = txRads - (aziumthMotorPositionInRadians * config.shooter.azimuthGearRatio);
            double txDegrees = txRads * 360.0 / (Math.PI * 2);

            // compute the vertical angle from our LL to the target
            double tyRads;
            double deltaH = VisionConstants.getTargetHeightInches() - VisionConstants.getCameraHeightInches();
            tyRads = Math.atan(deltaH / (yDistance / DriveConstants.METERS_PER_INCH));
            double tyDegrees = tyRads * 360.0 / (Math.PI * 2);
            tyDegrees -= VisionConstants.getCameraMountingAngle();

            if (limelightTable != null) {
                limelightTable.getEntry("tx").forceSetDouble(txDegrees);

                limelightTable.getEntry("ty").forceSetDouble(tyDegrees);

                if (Math.abs(txDegrees) < LIMELIGHT_VIEWING_ANGLE) {
                    limelightTable.getEntry("tv").forceSetNumber(1);
                } else {
                    limelightTable.getEntry("tv").forceSetNumber(0);
                }
            }
        }
    }

    /**
     * This event is triggered anytime the sim updates the outputs of the motors, basically every 1ms
     */
    void onMotorOutputsUpdated(MotorOutputs outputs) {
        if (robotInitialized) {
            for (MotorOutput motorOutput : outputs.getMotorOutputList()) {
                if (motorOutput.getId() == config.shooter.azimuth.id) {
                    aziumthMotorPositionInRadians = motorOutput.getSensorPosition();
                }
            }

        }
    }

    /**
     * Hook into the FieldRenderEvent to render our turret
     * @param fieldRenderEvent
     */
    private void onFieldRender(FieldRenderEvent fieldRenderEvent) {
        if (robotInitialized) {
            Graphics2D g2d = fieldRenderEvent.getG2d();
            Rectangle robotRect = fieldRenderEvent.getRobotRect();

            // our turret is a red circle in the middle of the robot
            int size = (int) (robotRect.width * .75);
            int x = robotRect.x + size / 4;
            int y = robotRect.y + size / 4;
            g2d.setColor(Color.RED);
            g2d.fillOval(x, y, size, size);
            g2d.setColor(Color.DARK_GRAY);
            g2d.drawOval(x, y, size, size);

            // put a dot where the robot thinks it is, with a line in the direction it's pointing
            Robot robot = (Robot)SimMain.getRobot();
            double posReportedX_m = robot.getDriveSubsystem().getPose().getTranslation().getX();
            double posReportedY_m = robot.getDriveSubsystem().getPose().getTranslation().getY();
            double posReportedHdg_rad = robot.getDriveSubsystem().getPose().getRotation().getRadians();
            double posReportedPolarR_m = Math.sqrt(Math.pow(posReportedX_m, 2.0) + Math.pow(posReportedY_m, 2.0));
            double posReportedPolarT_rad = Math.atan2(posReportedY_m, posReportedX_m);
            double posFieldPolarR_m = posReportedPolarR_m;
            double posFieldPolarT_rad = posReportedPolarT_rad + simulatorConfig.startPosition.heading + (Math.PI / 2);
            double posFieldHdg_rad = posReportedHdg_rad + simulatorConfig.startPosition.heading + (Math.PI / 2);
            double posFieldX_m = posFieldPolarR_m * Math.cos(posFieldPolarT_rad) + simulatorConfig.startPosition.x;
            double posFieldY_m = posFieldPolarR_m * Math.sin(posFieldPolarT_rad) + simulatorConfig.startPosition.y;
            final int dotRadius_px = 2;
            final int tailRadius_px = 10;
            int posGraphicsX_px = (int)(Field.imageSize.x) / 2 + (int) Math.round(posFieldX_m / Field.metersPerPixelWidth);
            int posGraphicsY_px = (int)(Field.imageSize.y) / 2 - (int) Math.round(posFieldY_m / Field.metersPerPixelHeight);
            int tailEndX_px = posGraphicsX_px + (int)(Math.cos(posFieldHdg_rad) * tailRadius_px);
            int tailEndY_px = posGraphicsY_px + (int)(Math.sin(posFieldHdg_rad) * tailRadius_px);
            g2d.setColor(Color.CYAN);
            g2d.fillOval(posGraphicsX_px - dotRadius_px, posGraphicsY_px - dotRadius_px, 2 * dotRadius_px, 2 * dotRadius_px);
            // currently reported theta is very wrong - so drawing the line is disabled as it's just a distraction
            // g2d.drawLine(posGraphicsX_px, posGraphicsY_px, tailEndX_px, tailEndY_px);

            // Draw a line showing the direction the turret is facing.
            g2d.setColor(Color.YELLOW);
            double turretHeading = (aziumthMotorPositionInRadians * config.shooter.azimuthGearRatio) + Math.PI + fieldRenderEvent.getHeading();
            int startX = x + size / 2;
            int startY = y + size / 2;
            int endX = startX + (int) (Math.cos(turretHeading - Math.PI / 2) * (robotRect.width / 2));
            int endY = startY + (int) (Math.sin(turretHeading - Math.PI / 2) * (robotRect.width / 2));
            g2d.drawLine(startX, startY, endX, endY);
        }
    }
}