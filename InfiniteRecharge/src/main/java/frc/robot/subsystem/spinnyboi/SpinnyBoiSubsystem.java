package frc.robot.subsystem.spinnyboi;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;



public class SpinnyBoiSubsystem extends BitBucketSubsystem {
    
    WPI_TalonSRX motor;
    
    public SpinnyBoiSubsystem(Config config) {
        super(config);
    }

    public void initialize() {
        super.initialize();
        motor = MotorUtils.makeSRX(config.spinnyboi.spinner);
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

    public void rotationControl(){
        //Rotate the wheel 3 - 5 times

        
    }

    public void postitionControl(){
        //Rotate the wheel to the color specified by the FMS
    }

}