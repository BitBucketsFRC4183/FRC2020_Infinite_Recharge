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
import frc.robot.subsystem.scoring.shooter.ball_management.BallManagementSubsystem;

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

    // Floats
    // TODO float rootBeer = good

    // Class Declarations
    RunningAverageFilter filter = new RunningAverageFilter(ShooterConstants.FILTER_LENGTH);
    public BallManagementSubsystem ballManagementSubsystem;

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
        ballManagementSubsystem = new BallManagementSubsystem(config);
        ballManagementSubsystem.initialize();
    }

    @Override
    public void initialize() {
        super.initialize();
        azimuthMotor = MotorUtils.makeSRX(config.shooter.azimuth);
        ballPropulsionMotor = MotorUtils.makeFX(config.shooter.shooter);
        feeder = MotorUtils.makeSRX(config.shooter.feeder);
        feeder.selectProfileSlot(MotorUtils.velocitySlot, 0);
        ballPropulsionMotor.selectProfileSlot(MotorUtils.velocitySlot, 0);
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Feeder Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity RPM", 60);
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
        // Put the outputs on the smart dashboard.
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Feeder Output", feeder.getMotorOutputPercent());
    }

    public void spinUp() {

        // Spin up the feeder.
        feeder.set(ControlMode.Velocity,
                MathUtils.unitConverter(SmartDashboard.getNumber(getName() + "/Feeder Velocity RPM", 60), 600, 8192));
        SmartDashboard.putString(getName() + "/Feeder State", "Feeding");

        // Spin up the shooter.
        ballPropulsionMotor.set(ControlMode.Velocity,
                MathUtils.unitConverter(SmartDashboard.getNumber(getName() + "/Shooter Velocity RPM", 60), 600, 8192)
                        / config.shooter.shooterGearRatio);
        SmartDashboard.putString(getName() + "/Shooter State", "Shooting");
    }

    public void stopSpinningUp() {
        // Spin up the feeder.
        feeder.set(0);
        SmartDashboard.putString(getName() + "/Feeder State", "Doing Nothing");

        // Spin up the shooter.
        ballPropulsionMotor.set(0);
        SmartDashboard.putString(getName() + "/Shooter State", "Doing Nothing");
    }

    public void fire() {
        ballManagementSubsystem.fire(
                (float) SmartDashboard.getNumber(getName() + "/BallManagementSubsystem/Output Percent", 50) / 100);
    }

    public void holdFire() {
        ballManagementSubsystem.doNotFire();
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
