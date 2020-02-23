package frc.robot.subsystem.climber;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends BitBucketSubsystem {
    public enum ClimbState {
        Extending, Retracting, Off;
    }

    protected WPI_TalonSRX motor1;
    protected WPI_TalonSRX motor2;
    private ClimbState climbState = ClimbState.Off;
    public ClimbSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        motor2.follow(motor1);
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
                motor1.set(0);
                SmartDashboard.putString(getName() + "/ClimbState", "Retracted");
                break;
    
            case Extending:
                motor1.set(SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.EXTEND_OUTPUT));
                SmartDashboard.putString(getName() + "/ClimbState", "Extending");
                if (motor1.getStatorCurrent() < 1) {
                    climbState = ClimbState.Off;
                }
                break;

                //no holding state is needed, since a mechanical ratchet will hold the robot while its hanging
            case Retracting:
                motor1.set(SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.RETRACT_OUTPUT));
                SmartDashboard.putString(getName() + "/ClimbState", "Retracting");
                break;
            }
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
            }


    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
    }
}