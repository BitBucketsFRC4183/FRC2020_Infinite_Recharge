package frc.robot.subsystem.copypaste;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

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

	public void testInit() {

    }
	
	public void testPeriodic() {

    }
	
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
    }

    @Override
    public BaseTalon[] getTalons() {
        // TODO Auto-generated method stub
        return null;
    }

}