package frc.robot.subsystem.drive;

import java.util.HashMap;

import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.Subsystems;



public class DriveSubsystem extends BitBucketSubsystem {
    public DriveSubsystem() {
        setName("DriveSubsystem");
    }



    public void initialize() {
        initializeBaseDashboard();
    }

	public void diagnosticsInitialize() {

    }
	
	public void diagnosticsPeriodic() {

    }
	
	public void diagnosticsCheck() {

    }
	
	@Override
    protected void initDefaultCommand() {

    }
    
    @Override
    public void periodic() {
        updateBaseDashboard();
    }




    @Override
	public void injectDependencies(HashMap<Subsystems, BitBucketSubsystem> deps) {

    };

    @Override
	public Subsystems getSubsystemID() {
        return Subsystems.DRIVE;
    };
}