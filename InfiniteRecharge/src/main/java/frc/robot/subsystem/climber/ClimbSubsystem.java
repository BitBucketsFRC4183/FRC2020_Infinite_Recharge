package frc.robot.subsystem.climber;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;

public class ClimbSubsystem extends BitBucketSubsystem {

    public ClimbSubsystem(Config config) {
        super(config);
    
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