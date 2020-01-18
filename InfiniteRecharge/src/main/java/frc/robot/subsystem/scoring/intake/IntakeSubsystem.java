package frc.robot.subsystem.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class IntakeSubsystem extends BitBucketSubsystem {

    protected WPI_TalonSRX motor;
    boolean intaking;

    public IntakeSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        
        initializeBaseDashboard();
        // motor = new WPI_TalonSRX(config.shooter.intake.id);
        // MotorUtils.motorInit(motor, config.shooter.intake);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Intake Speed", 0.2);
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
    public void periodic(float deltaTime) {
        if (intaking) {
            motor.set(SmartDashboard.getNumber(getName() + "/Intake Speed", 0.2));
            SmartDashboard.putString(getName() + "/IntakeState", "Intaking");
        } else {
            motor.set(0);
            SmartDashboard.putString(getName() + "/IntakeState", "Not Intaking");
        }
        SmartDashboard.putNumber(getName() + "/IntakeOut", motor.getMotorOutputPercent());
    }

    public void intake() {
        intaking = true;
    }

    public void doNotIntake() {
        intaking = false;
    }

}