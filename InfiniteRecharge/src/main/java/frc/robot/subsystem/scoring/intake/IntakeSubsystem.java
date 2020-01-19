package frc.robot.subsystem.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public IntakeSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();

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

    public void barDown() {

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

}