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

import frc.robot.utils.data.filters.RunningAverageFilter;
import frc.robot.subsystem.scoring.shooter.ShooterConstants;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Variables

    // Booleans
    public boolean shooting = false;
    public boolean feeding = false;
    public boolean feederVelocityControl = false;
    public boolean shooterVelocityControl = false;

    // Integers
    public int targetPosition;
    public int targetChange;

    // Class Declarations
    RunningAverageFilter filter = new RunningAverageFilter(ShooterConstants.FILTER_LENGTH);

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
        azimuthMotor.setSensorPhase(ShooterConstants.USE_SENSOR_PHASE);
        ballPropulsionMotor = MotorUtils.makeFX(config.shooter.shooter);
        feeder = MotorUtils.makeSRX(config.shooter.feeder);
        feeder.selectProfileSlot(MotorUtils.velocitySlot, 0);
        ballPropulsionMotor.selectProfileSlot(MotorUtils.velocitySlot, 0);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Feeder Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity RPM", 500);
        SmartDashboard.putNumber(getName() + "/Feeder Velocity RPM", 60);
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
        if (shooting && !shooterVelocityControl) {

            ballPropulsionMotor.set(SmartDashboard.getNumber(getName() + "/Shooter Output Percent", 0.2));
            SmartDashboard.putString(getName() + "/Shooter State", "Shooting with percent output");

        } else if (shooting && shooterVelocityControl) {

            ballPropulsionMotor.set(ControlMode.Velocity,
                    MathUtils.unitConverter(SmartDashboard.getNumber(getName() + "/Shooter Velocity RPM", 60), 600,
                            8192) / config.shooter.shooterGearRatio);
            SmartDashboard.putString(getName() + "/Shooter State", "Shooting with velocity control");
            SmartDashboard.putNumber(getName() + "/Velocity Error");

        } else {

            ballPropulsionMotor.set(0);
            SmartDashboard.putString(getName() + "/Shooter State", "Not shooting");

        }
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Shooter Velocity Output",
                ballPropulsionMotor.getSelectedSensorVelocity());

        //

        targetPosition = (int) (targetPosition + (targetChange * deltaTime));
        azimuthMotor.set(ControlMode.MotionMagic, targetPosition);

        if (feeding && !feederVelocityControl) {

            feeder.set(SmartDashboard.getNumber(getName() + "/Feeder Output Percent", 0.2));
            SmartDashboard.putString(getName() + "/FeederState", "Feeding with percent output");

        } else if (feeding && feederVelocityControl) {

            feeder.set(ControlMode.Velocity, MathUtils
                    .unitConverter(SmartDashboard.getNumber(getName() + "/Feeder Velocity RPM", 60), 600, 8192));
            SmartDashboard.putString(getName() + "/FeederState", "Feeding with with velocity control");

        } else {

            feeder.set(0);
            SmartDashboard.putString(getName() + "/FeederState", "Not Feeding");

        }
        SmartDashboard.putNumber(getName() + "/Feeder Output", feeder.getMotorOutputPercent());
    }

    public void feed(boolean hasVelocityControl) {
        feeding = true;
        if (hasVelocityControl) {
            feederVelocityControl = true;
        } else {
            feederVelocityControl = false;
        }
    }

    public void doNotFeed() {
        feeding = false;
    }

    public void shoot(boolean hasVelocityControl) {
        shooting = true;
        if (hasVelocityControl) {
            shooterVelocityControl = true;
        } else {
            shooterVelocityControl = false;
        }
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
                / config.shooter.azimuthGearRatio;

        // Target position changes by this number every time periodic is called.
        targetChange = (int) (smartDashboardTurnRateTicks * spinRate);
    }

    public void rotateToDeg(double targetPoint) {
        double targetPointTicks = MathUtils.unitConverter(targetPoint, 360, config.shooter.azimuth.ticksPerRevolution)
                / config.shooter.azimuthGearRatio;
        targetPosition = (int) (targetPointTicks);
        targetChange = 0;
    }

    public void rotateByDeg(double degrees) {
        rotateToDeg(getTargetTurretDegGivenOffset(degrees));
    }

    public double getTurretDeg() {
        double encoderDeg = MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                config.shooter.azimuth.ticksPerRevolution, 360.0);
        double turretDeg = encoderDeg * config.shooter.azimuthGearRatio;
        return turretDeg;
    }

    /*
     * Returns target degrees of turret given an offset
     */
    public double getTargetTurretDegGivenOffset(double offset) {
        return getTurretDeg() + offset;
    }

    public void rotateTurretGivenLLOffset(double offset) {
        double degrees = getTargetTurretDegGivenOffset(offset);
        // The offset and thus the degrees might change, causing the robot to oscillate
        // about its target. To prevent this, take an average.
        // If enabled in the constants file, calculate the average of the last values
        // passed in (up to 25, configurable in ShooterConstants.java).
        degrees = ShooterConstants.USE_FILTER ? filter.calculate(degrees) : degrees;
        rotateToDeg(degrees);
    }

}
