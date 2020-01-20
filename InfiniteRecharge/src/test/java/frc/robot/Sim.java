package frc.robot;

import java.io.IOException;

import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.simulator.SimMain;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs;
import frc.robot.simulator.network.proto.RobotProto.MotorOutputs.MotorOutput;
import frc.robot.simulator.sim.events.EventManager;
import frc.robot.simulator.sim.events.RobotInitializedEvent;

public class Sim {

    public static void main(String[] args) throws InterruptedException, IOException {
        // create a new BitBuckets sim object to respond to simulator events
        Sim bitbucketsSim = new Sim();

        // subscribe to events from the simulator
        EventManager.subscribeToRobotInitializedEvents(bitbucketsSim::onRobotInitialized);
        EventManager.subscribeToMotorOutputsEvents(bitbucketsSim::onMotorOutputsUpdated);
        
        // start the sim
        SimMain.main(args);
    }

    private final Config config;
    private boolean robotInitialized;

    /**
     * Initialize the sim with a config for our robot, so we know what motor ids to use
     */
    public Sim() {
        config = ConfigChooser.getConfig();
    }

    /**
     * This is called when the robotInit() is complete, so we know we can start checking motor outputs and configs.
     * @param robotInitializedEvent
     */
    void onRobotInitialized(RobotInitializedEvent robotInitializedEvent) {
        robotInitialized = true;

        // TODO: do limelight network table setup here
    }

    void onMotorOutputsUpdated(MotorOutputs outputs) {
        if (robotInitialized) {
            for (MotorOutput motorOutput : outputs.getMotorOutputList()) {
                if (motorOutput.getId() == config.shooter.azimuth.id) {
                    double aziumthPositionInRadians = motorOutput.getSensorPosition();
                    // TODO: Update limelight values to account for new motor output
                }
            }

        }
    }


}