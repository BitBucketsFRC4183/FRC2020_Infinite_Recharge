package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
    public boolean feeding = false;

    // Integers
    int targetPosition;
    int targetChange;

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    protected WPI_TalonSRX azimuthMotor;
    protected WPI_TalonFX ballPropulsionMotor;
    protected WPI_TalonSRX feeder;

    // Neos

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        azimuthMotor = MotorUtils.makeSRX(config.shooter.azimuth);
        ballPropulsionMotor = MotorUtils.makeFX(config.shooter.shooter);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Velocity", 0.2);
        SmartDashboard.putNumber(getName() + "/Feeder Speed", 0.2);
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
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());

        targetPosition = (int) (targetPosition + (targetChange * deltaTime));
        azimuthMotor.set(ControlMode.MotionMagic, targetPosition);

        if (feeding) {
            feeder.set(SmartDashboard.getNumber(getName() + "/Feeder Speed", 0.2));
            SmartDashboard.putString(getName() + "/FeederState", "Feeding");
        } else {
            feeder.set(0);
            SmartDashboard.putString(getName() + "/FeederState", "Not Feeding");
        }
        SmartDashboard.putNumber(getName() + "/FeederOut", feeder.getMotorOutputPercent());
    }

    public void feed() {
        feeding = true;
    }

    public void doNotFeed() {
        feeding = false;
    }

    public void shoot() {
        shooting = true;
    }

    public void doNotShoot() {
        shooting = false;
    }

    public void rotate(double spinRate) {
        // Turn turret at a quantity of degrees per second configurable in the smart
        // dashboard.
        double smartDashboardTurnRateTicks = MathUtils
                .unitConverter(SmartDashboard.getNumber(getName() + "/Turret Turn Rate",
                        config.shooter.defaultTurnVelocityDeg), 360, config.shooter.azimuth.ticksPerRevolution)
                / config.shooter.gearRatio;

        // Target position changes by this number every time periodic is called.
        targetChange = (int) (smartDashboardTurnRateTicks * spinRate);
    }

    public void rotateToDeg(double targetPoint) {
        double targetPointTicks = MathUtils.unitConverter(targetPoint, 360, config.shooter.azimuth.ticksPerRevolution);
        targetPosition = (int) (targetPointTicks);
        targetChange = 0;
    }

}