package frc.robot.subsystem.climber;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends BitBucketSubsystem {
    public enum ClimbState {
        Extending, Retracting, Off;
    }

    protected WPI_TalonSRX motorRight;
    protected WPI_TalonSRX motorLeft;
    private ClimbState climbState = ClimbState.Off;
    public ClimbSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        motorRight = MotorUtils.makeSRX(config.climb.climbRight);
        motorLeft = MotorUtils.makeSRX(config.climb.climbLeft);
        motorLeft.setInverted(true);
        motorLeft.follow(motorRight);
    }
    
    @Override
	public void testInit() {
       
    }
    
    @Override
	public void testPeriodic() {

    }
    
    @Override
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        
        switch (climbState) {
            case Off:
                motorRight.set(0);
                SmartDashboard.putString(getName() + "/ClimbState", "Retracted");
                break;
    
            case Extending:
                motorRight.set(SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.EXTEND_OUTPUT));
                SmartDashboard.putString(getName() + "/ClimbState", "Extending");
                break;

                //no holding state is needed, since a mechanical ratchet will hold the robot while its hanging
            case Retracting:
                motorRight.set(SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.RETRACT_OUTPUT));
                SmartDashboard.putString(getName() + "/ClimbState", "Retracting");
                break;
            }
        }
    public boolean isExtending() {
        return climbState == ClimbState.Extending ;
    }
        
    public void off() {
        climbState = ClimbState.Off;
    }

    public void retracting() {
        climbState = ClimbState.Retracting;
    }

    public void extending() {
        climbState = ClimbState.Extending;
    }


    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
    }

	@Override
	public BaseTalon[] getTalons() {
		return new BaseTalon[] {motorRight, motorLeft};
	}
}