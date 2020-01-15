package frc.robot.subsystem.copypaste;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;



public class TestSubsystem extends BitBucketSubsystem {
    public TestSubsystem(Config config) {
        super(config);

        setName("Test Subsystem");
    }



    public void initialize() {
        super.initialize();
    }

	public void diagnosticsInitialize() {

    }
	
	public void diagnosticsPeriodic() {

    }
	
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();
    }

}