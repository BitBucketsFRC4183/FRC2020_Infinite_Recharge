package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.operatorinterface.PS4Constants;
import frc.robot.subsystem.BitBucketSubsystem;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Talons
    protected WPI_TalonSRX azimuthMotor;

    //////////////////////////////////////////////////////////////////////////////
    // Neos
    protected CANSparkMax ballPropulsionMotor;

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        azimuthMotor = new WPI_TalonSRX(config.shooter.azimuth.id);
        ballPropulsionMotor = new CANSparkMax(config.shooter.shooter.id, CANSparkMaxLowLevel.MotorType.kBrushless);
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
        // TODO Auto-generated method stub
        // if(OI.driverControl.getRawButton(PS4Constants.SQUARE.getValue())){
        //     ballPropulsionMotor.set(SmartDashboard.getNumber(getName() + "/Shooter Velocity", 0.2));
        // }
    }

}