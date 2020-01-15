package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.talonutils.MotorUtils;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Variables

    // Booleans
    public boolean shooting = false;

    // Integers
    int targetPosition;
    int targetChange;

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    protected WPI_TalonSRX azimuthMotor;

    // Neos
    protected CANSparkMax ballPropulsionMotor;

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        azimuthMotor = new WPI_TalonSRX(config.shooter.azimuth.id);
        ballPropulsionMotor = new CANSparkMax(config.shooter.shooter.id, CANSparkMaxLowLevel.MotorType.kBrushless);

        MotorUtils.motorInit(azimuthMotor, config.shooter.azimuth);
        MotorUtils.motorInit(ballPropulsionMotor, config.shooter.shooter);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Velocity", 0.2);
        SmartDashboard.putNumber(getName() + "/Turret Turn Rate", config.shooter.defaultTurnVelocityDeg);
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
        if (shooting) {
            ballPropulsionMotor.set(SmartDashboard.getNumber(getName() + "/Shooter Velocity", 0.2));
            SmartDashboard.putString(getName() + "/Shooter State", "shooting");
        } else {
            ballPropulsionMotor.set(0);
            SmartDashboard.putString(getName() + "/Shooter State", "not shooting");
        }
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getAppliedOutput());

        targetPosition = (int) (targetPosition + (targetChange * deltaTime));
        azimuthMotor.set(ControlMode.MotionMagic, targetPosition);
    }

    public void shoot() {
        shooting = true;
    }

    public void doNotShoot() {
        shooting = false;
    }

    public void rotate(double spinRate) {
        // Turn turret at a quantity of degrees per second configurable in the smart dashboard.
        double smartDashboardTurnRateTicks = MathUtils
                .unitConverter(SmartDashboard.getNumber(getName() + "/Turret Turn Rate",
                        config.shooter.defaultTurnVelocityDeg), 360, config.shooter.azimuthMotorTicks)
                / config.shooter.gearRatio;

        // Target position changes by this number every time periodic is called.
        targetChange = (int) (smartDashboardTurnRateTicks * spinRate);
    }

}