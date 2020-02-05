package frc.robot;

import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.simulator.SimMain;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs.MotorOutput;
import frc.robot.simulator.sim.RobotPosition;
import frc.robot.simulator.sim.RobotPosition.Type;
import frc.robot.simulator.sim.events.EventManager;
import frc.robot.simulator.sim.events.RobotInitializedEvent;

public class Sim {

    // the coordinates of the blue target, in meters, on the field
    // this is based on the 0, 0 point being the center of the field image.
    private double targetX = 1.42;
    private double targetY = 8.35;
    private double aziumthMotorPositionInRadians = 0;

    public static void main(String[] args) throws InterruptedException, IOException {
        // create a new BitBuckets sim object to respond to simulator events
        Sim bitbucketsSim = new Sim();

        // subscribe to events from the simulator
        EventManager.subscribeToRobotInitializedEvents(bitbucketsSim::onRobotInitialized);
        EventManager.subscribeToMotorOutputsEvents(bitbucketsSim::onMotorOutputsUpdated);
        EventManager.subscribeToRobotPositionEvents(bitbucketsSim::onRobotPositionUpdated);

        // start the sim
        SimMain.main(args);
    }

    private final Config config;
    private boolean robotInitialized;
    private NetworkTable limelightTable;

    /**
     * Initialize the sim with a config for our robot, so we know what motor ids to
     * use
     */
    public Sim() {
        config = ConfigChooser.getConfig();
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

        // TODO: do limelight network table setup here
    }

    void onRobotPositionUpdated(RobotPosition robotPosition) {
        if (robotPosition.type == Type.WheelDisplacement) {
            double yDistance = targetY - robotPosition.y;
            double xDistance = targetX - robotPosition.x;

            // compute the angle (to the right) that the target is from our robot center
            double txRads;
            if (xDistance > 0) {
                txRads = (Math.PI / 2) - Math.atan(yDistance / xDistance);
            } else if (xDistance < 0) {
                txRads = -((Math.PI / 2) + Math.atan(yDistance / xDistance));
            } else {
                txRads = 0;
            }

            // if our robot is rotated to the right, update the tx by that offset
            txRads = txRads - robotPosition.heading;

            // if our turrent is rotated, update txRads by the azimuth change
            txRads = txRads - (aziumthMotorPositionInRadians * config.shooter.azimuthGearRatio);
            double txDegrees = txRads * 360.0 / (Math.PI * 2);

            if (limelightTable != null) {
                limelightTable.getEntry("tx").forceSetDouble(txDegrees);

                // TODO: Figure out ty based on distance to target

                if (Math.abs(txDegrees) < 45) {
                    limelightTable.getEntry("tv").forceSetNumber(1);
                } else {
                    limelightTable.getEntry("tv").forceSetNumber(0);
                }
            }
        }
    }

    void onMotorOutputsUpdated(MotorOutputs outputs) {
        if (robotInitialized) {
            for (MotorOutput motorOutput : outputs.getMotorOutputList()) {
                if (motorOutput.getId() == config.shooter.azimuth.id) {
                    aziumthMotorPositionInRadians = motorOutput.getSensorPosition();
                }
            }

        }
    }

}