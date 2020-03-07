package frc.robot.subsystem.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class IntakeSubsystem extends BitBucketSubsystem {
    public enum IntakeState {
        Intaking, Outaking, Off;
    }

    //
    // Change this type to pick config
    //
    private IntakeState intakeState = IntakeState.Off;

    protected WPI_TalonSRX motor;
    private DoubleSolenoid intakePivet;

    public IntakeSubsystem(Config config) {
        super(config);
    }
    
      

    @Override
    public void initialize() {
        super.initialize();
        if (config.intake.intakePivotEnabled){
            intakePivet = new DoubleSolenoid(RobotMap.INTAKE_PNEUMATIC_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMATIC_CLOSED_CHANNEL); 
            intakePivet.set(Value.kForward);
            SmartDashboard.putString(getName() + "/Intake Pivet", "Enabled");
        } else {
            SmartDashboard.putString(getName() + "/Intake Pivet", "Disabled");
        }
        motor = MotorUtils.makeSRX(config.intake.intake);
        
        dashboardInit();
        
    }

    @Override
    public void dashboardInit() {
        // TODO Auto-generated method stub
        super.dashboardInit();
        SmartDashboard.putNumber(getName() + "/Intake Speed", IntakeConstants.INTAKE_OUTPUT);
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
        switch (intakeState) {

        case Off:
            motor.set(0);
            SmartDashboard.putString(getName() + "/IntakeState", "Not Intaking");
            break;

        case Intaking:
            motor.set(SmartDashboard.getNumber(getName() + "/Intake Speed", IntakeConstants.INTAKE_OUTPUT));
            SmartDashboard.putString(getName() + "/IntakeState", "Intaking");
            break;

        case Outaking:
            motor.set(-SmartDashboard.getNumber(getName() + "/Intake Speed", IntakeConstants.OUTAKE_OUTPUT));
            SmartDashboard.putString(getName() + "/IntakeState", "Outaking");
            break;
        }
    }

    public void intake() {
        intakeState = IntakeState.Intaking;
    }

    public void outake() {
        intakeState = IntakeState.Outaking;
    }

    public void off() {
        intakeState = IntakeState.Off;
    }

    public void toggleIntakeArm(){
        if(config.intake.intakePivotEnabled){
            Value armState = intakePivet.get();
            if (armState == Value.kForward) {
                armState = Value.kReverse;
            }
            else if (armState == Value.kReverse){
                armState = Value.kForward;
            }
            else if (armState == Value.kOff){
                armState = Value.kForward;
            }
            intakePivet.set(armState);
            }
        
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        SmartDashboard.putNumber(getName() + "/IntakeOut", motor.getMotorOutputPercent());
    }

    public void disable(){
        motor.set(0);
    }



    @Override
    public void listTalons() {
        talons.add(motor);
    }
}