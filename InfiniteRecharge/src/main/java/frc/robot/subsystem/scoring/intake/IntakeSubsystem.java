package frc.robot.subsystem.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.operatorinterface.PS4Constants;
import frc.robot.subsystem.BitBucketSubsystem;

public class IntakeSubsystem extends BitBucketSubsystem {

    protected WPI_TalonSRX motor;
    boolean intaking;

    public IntakeSubsystem(Config config) {
        super(config);
        // TODO Auto-generated constructor stub
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        motor = new WPI_TalonSRX(config.shooter.intake.id);

    }

    @Override
    public void diagnosticsInitialize() {
        // TODO Auto-generated method stub

    }

    @Override
    public void diagnosticsPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void diagnosticsCheck() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {        
        if(intaking){
            motor.set(SmartDashboard.getNumber(getName() + "/Intake Speed", 0.2));
        } else {
            motor.set(0);
        }
    }
    public void intake() {
        intaking = true;
    }
    public void doNotIntake() {
        intaking = false;
    }

}