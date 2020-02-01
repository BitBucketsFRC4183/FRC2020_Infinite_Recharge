package frc.robot.subsystem.scoring.intake;

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
        intakePivet = new DoubleSolenoid(RobotMap.INTAKE_PNEUMATIC_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMATIC_CLOSED_CHANNEL); 
        intakePivet.set(Value.kReverse);
        initializeBaseDashboard();
        motor = MotorUtils.makeSRX(config.intake.intake);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Intake Speed", 0.2);
    }

    @Override
    public void diagnosticsPeriodic() {

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
            motor.set(SmartDashboard.getNumber(getName() + "/Intake Speed", 0.2));
            SmartDashboard.putString(getName() + "/IntakeState", "Intaking");
            break;

        case Outaking:
            motor.set(-SmartDashboard.getNumber(getName() + "/Intake Speed", 0.2));
            SmartDashboard.putString(getName() + "/IntakeState", "Outaking");
            break;
        }
        SmartDashboard.putNumber(getName() + "/IntakeOut", motor.getMotorOutputPercent());
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