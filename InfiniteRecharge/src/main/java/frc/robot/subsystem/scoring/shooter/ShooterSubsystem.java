package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    public boolean validTarget = false;

    // Integers
    public int targetPositionAzimuth;
    public int targetChangeAzimuth;

    public int targetPositionElevation;
    public int targetChangeElevation;

    // Floats
    // TODO float rootBeer = good
    public double forwardAzimuthSoftLimit;
    public double backwardAzimuthSoftLimit;

    public double forwardElevationSoftLimit;
    public double backwardElevationSoftLimit;

    // Doubles
    public double degreesToRotateAzimuth = 0.0;
    public double degreesToRotateElevation = 0.0;

    // Class Declarations
    RunningAverageFilter filter = new RunningAverageFilter(ShooterConstants.FILTER_LENGTH);
    public BallManagementSubsystem ballManagementSubsystem;

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    protected WPI_TalonSRX azimuthMotor;
    protected WPI_TalonSRX elevationMotor;
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
        elevationMotor = MotorUtils.makeSRX(config.shooter.elevation);

        ballPropulsionMotor = MotorUtils.makeFX(config.shooter.shooter);
        feeder = MotorUtils.makeSRX(config.shooter.feeder);
        feeder.selectProfileSlot(MotorUtils.velocitySlot, 0);

        ballPropulsionMotor.selectProfileSlot(MotorUtils.velocitySlot, 0);
        ballManagementSubsystem = new BallManagementSubsystem(config);
        ballManagementSubsystem.initialize();

        forwardAzimuthSoftLimit = MathUtils.unitConverter(config.shooter.forwardAzimuthSoftLimit, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;
        backwardAzimuthSoftLimit = MathUtils.unitConverter(config.shooter.backwardAzimuthSoftLimit, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        forwardElevationSoftLimit = MathUtils.unitConverter(config.shooter.forwardElevationSoftLimit, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;
        backwardElevationSoftLimit = MathUtils.unitConverter(config.shooter.backwardElevationSoftLimit, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        if (config.shooter.forwardAzimuthSoftLimit == -1 || config.shooter.backwardAzimuthSoftLimit == -1) {
            azimuthMotor.configForwardSoftLimitEnable(true);
            azimuthMotor.configForwardSoftLimitThreshold((int) forwardAzimuthSoftLimit);

            azimuthMotor.configReverseSoftLimitEnable(true);
            azimuthMotor.configReverseSoftLimitThreshold((int) -backwardAzimuthSoftLimit);
        }
        if (config.shooter.forwardElevationSoftLimit == -1 || config.shooter.backwardElevationSoftLimit == -1) {
            elevationMotor.configForwardSoftLimitEnable(true);
            elevationMotor.configForwardSoftLimitThreshold((int) forwardElevationSoftLimit);

            elevationMotor.configReverseSoftLimitEnable(true);
            elevationMotor.configReverseSoftLimitThreshold((int) -backwardElevationSoftLimit);
        }
    }

    @Override
    public void diagnosticsInitialize() {
        SmartDashboard.putNumber(getName() + "/Shooter Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Feeder Output Percent", 0.2);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity RPM", 500);
        SmartDashboard.putNumber(getName() + "/Feeder Velocity RPM", 60);
        SmartDashboard.putNumber(getName() + "/Azimuth Turn Rate", config.shooter.defaultAzimuthTurnVelocityDeg);
        SmartDashboard.putNumber(getName() + "/Elevation Turn Rate", config.shooter.defaultAzimuthTurnVelocityDeg);
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

        autoAim();

        targetPositionAzimuth = (int) (targetPositionAzimuth + (targetChangeAzimuth * deltaTime));
        targetPositionElevation = (int) (targetPositionElevation + (targetChangeElevation * deltaTime));

        if (forwardAzimuthSoftLimit != -1 && backwardAzimuthSoftLimit != -1) {
            if (targetPositionAzimuth > forwardAzimuthSoftLimit) {
                targetPositionAzimuth = (int) forwardAzimuthSoftLimit;
            } else if (targetPositionAzimuth < -backwardAzimuthSoftLimit) {
                targetPositionAzimuth = (int) backwardAzimuthSoftLimit;
            }
        }
        if (forwardElevationSoftLimit != -1 && backwardElevationSoftLimit != -1) {
            if (targetPositionElevation > forwardElevationSoftLimit) {
                targetPositionElevation = (int) forwardElevationSoftLimit;
            } else if (targetPositionAzimuth < -backwardElevationSoftLimit) {
                targetPositionElevation = (int) backwardElevationSoftLimit;
            }
        }

        azimuthMotor.set(ControlMode.MotionMagic, targetPositionAzimuth);
        elevationMotor.set(ControlMode.MotionMagic, targetPositionElevation);

        // Put the outputs on the smart dashboard.
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Feeder Output", feeder.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Shooter Velocity Output",
                ballPropulsionMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(getName() + "/Degrees to Rotate", degreesToRotateAzimuth);
        SmartDashboard.putNumber(getName() + "/Target Position ", targetPositionAzimuth);
        SmartDashboard.putBoolean(getName() + "/Valid Target ", validTarget);
        SmartDashboard.putNumber(getName() + "/Azimuth Position ", azimuthMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber(getName() + "/Azimuth Target Position Deg ",
                MathUtils.unitConverter(targetPositionAzimuth, config.shooter.azimuth.ticksPerRevolution, 360)
                        * config.shooter.azimuthGearRatio);

        SmartDashboard.putNumber(getName() + "/Azimuth Position Deg ",
                MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                        config.shooter.azimuth.ticksPerRevolution, 360) * config.shooter.azimuthGearRatio);
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

    public void rotate(double spinRateAzimuth, double spinRateElevation) {
        // Turn turret at a quantity of degrees per second configurable in the smart
        // dashboard.
        double smartDashboardTurnRateTicksAzimuth = MathUtils
                .unitConverter(
                        SmartDashboard.getNumber(getName() + "/Azimuth Turn Rate",
                                config.shooter.defaultAzimuthTurnVelocityDeg),
                        360, config.shooter.azimuth.ticksPerRevolution)
                / config.shooter.azimuthGearRatio;

        double smartDashboardTurnRateTicksElevation = MathUtils.unitConverter(
                SmartDashboard.getNumber(getName() + "/Elevation Turn Rate",
                        config.shooter.defaultElevationTurnVelocityDeg),
                360, config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        // Target position changes by this number every time periodic is called.
        targetChangeAzimuth = (int) (smartDashboardTurnRateTicksAzimuth * spinRateAzimuth);
        targetChangeElevation = (int) (smartDashboardTurnRateTicksElevation * spinRateElevation);
    }

    public void rotateToDeg(double targetPointAzimuth, double targetPointElevation) {
        double targetPointTicksAzimuth = MathUtils.unitConverter(targetPointAzimuth, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        double targetPointTicksElevation = MathUtils.unitConverter(targetPointElevation, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        targetPositionAzimuth = (int) (targetPointTicksAzimuth);
        targetChangeAzimuth = 0;

        targetPositionElevation = (int) (targetPointTicksElevation);
        targetChangeElevation = 0;
    }

    public void rotateByDeg(double degreesAzimuth, double degreesElevation) {
        rotateToDeg(getTargetAzimuthDegGivenOffset(degreesAzimuth), getTargetElevationDegGivenOffset(degreesElevation));
    }

    public double getAzimuthDeg() {
        double encoderDeg = MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                config.shooter.azimuth.ticksPerRevolution, 360.0);
        double turretDeg = encoderDeg * config.shooter.azimuthGearRatio;
        return turretDeg;
    }

    public double getElevationDeg() {
        double encoderDeg = MathUtils.unitConverter(elevationMotor.getSelectedSensorPosition(),
                config.shooter.elevation.ticksPerRevolution, 360.0);
        double turretDeg = encoderDeg * config.shooter.elevationGearRatio;
        return turretDeg;
    }

    /*
     * Returns target degrees of turret given an offset
     */
    public double getTargetAzimuthDegGivenOffset(double offset) {
        return getAzimuthDeg() + offset;
    }

    public double getTargetElevationDegGivenOffset(double offset) {
        return getElevationDeg() + offset;
    }

    public void rotateTurretWithLLOffset() {
        rotateToDeg(degreesToRotateAzimuth, degreesToRotateElevation);
    }

    public void autoAim() {
        double defaultVal = 0;

        NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
        tableInstance.startClientTeam(4183);

        NetworkTable limelightTable = tableInstance.getTable("limelight");
        double tx = limelightTable.getEntry("tx").getDouble(defaultVal);
        double ty = limelightTable.getEntry("ty").getDouble(defaultVal);

        double tv = limelightTable.getEntry("tv").getDouble(defaultVal);
        if (tv == 1) {
            validTarget = true;
        } else {
            validTarget = false;
        }

        if (validTarget) {
            double degrees = getTargetAzimuthDegGivenOffset(tx);
            // The offset and thus the degrees might change, causing the robot to oscillate
            // about its target. To prevent this, take an average.
            // If enabled in the constants file, calculate the average of the last values
            // passed in (up to 25, configurable in ShooterConstants.java).
            degreesToRotateAzimuth = ShooterConstants.USE_FILTER ? filter.calculate(degrees) : degrees;
        }
    }

}
