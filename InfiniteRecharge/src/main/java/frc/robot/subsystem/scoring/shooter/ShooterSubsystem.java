package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Booleans
    public boolean shooting = false;

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
        initializeBaseDashboard();
        azimuthMotor = new WPI_TalonSRX(config.shooter.azimuth.id);
        ballPropulsionMotor = new CANSparkMax(config.shooter.shooter.id, CANSparkMaxLowLevel.MotorType.kBrushless);

        MotorUtils.motorInit(azimuthMotor, config.shooter.azimuth);
        MotorUtils.motorInit(ballPropulsionMotor, config.shooter.shooter);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Velocity", 0.2);
        SmartDashboard.putNumber(getName() + "/Turret Speed Multiplier", 0.1);
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
        if (shooting) {
            ballPropulsionMotor.set(SmartDashboard.getNumber(getName() + "/Shooter Velocity", 0.2));
            SmartDashboard.putString(getName() + "/Shooter State", "shooting");
        } else {
            ballPropulsionMotor.set(0);
            SmartDashboard.putString(getName() + "/Shooter State", "not shooting");
        }
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getAppliedOutput());
    }

    public void shoot() {
        shooting = true;
    }

    public void doNotShoot() {
        shooting = false;
    }

    public void rotate(double spinRate) {

        double finalSpinRate = spinRate * SmartDashboard.getNumber(getName() + "/Turret Speed Multiplier", 0.1);

        azimuthMotor.set(finalSpinRate);

    }

}