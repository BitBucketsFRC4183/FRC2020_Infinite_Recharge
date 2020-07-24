package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.talonutils.MotorUtils;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.utils.data.filters.RunningAverageFilter;
import frc.robot.subsystem.scoring.shooter.ball_management.BallManagementSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterCalculator;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Variables

    // Booleans
    public boolean shooting = false;
    public boolean feeding = false;
    public boolean feederVelocityControl = false;
    public boolean shooterVelocityControl = false;

    private boolean upToSpeed = false;
    private boolean positionElevationSwitcherAlreadyPressed = false;
    private boolean spinningUp = false;
    private boolean autoAiming = false; // used to decide which velocity to use for the shooter

    private boolean leftAzimuthLimitSwitchClosed;
    private boolean rightAzimuthLimitSwitchClosed;
    private boolean reverseElevationLimitSwitchClosed;

    // Integers

    @Log(name = "Azimuth Target Position (Ticks)")
    private int targetPositionAzimuth_ticks;

    private int targetChangeAzimuth_ticks;

    @Log(name = "Elevation Target Position (Ticks)")
    private int targetPositionElevation_ticks;

    private int targetChangeElevation_ticks;

    // Floats
    // TODO float rootBeer = good
    private float[] positions = config.shooter.elevationPositions_deg;

    // Doubles
    @Log
    private double absoluteDegreesToRotateAzimuth = 0.0;
    private double shooterVelocity_ticks = 0.0;

    private double rightAzimuthSoftLimit_ticks;
    private double leftAzimuthSoftLimit_ticks;

    private double forwardElevationSoftLimit_ticks;
    private double backwardElevationSoftLimit_ticks;

    // Class Declarations
    RunningAverageFilter azimuthFilter = new RunningAverageFilter(config.shooter.FILTER_LENGTH);
    RunningAverageFilter elevationFilter = new RunningAverageFilter(config.shooter.FILTER_LENGTH);
    RunningAverageFilter feederFilter = new RunningAverageFilter(config.shooter.FEEDER_FILTER_LENGTH);

    @Log.Exclude
    @io.github.oblarg.oblog.annotations.Config.Exclude
    public BallManagementSubsystem ballManagementSubsystem;

    @Log.Exclude
    @io.github.oblarg.oblog.annotations.Config.Exclude
    private VisionSubsystem visionSubsystem;
    private final ShooterCalculator shooterCalculator;

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    private WPI_TalonSRX azimuthMotor;
    private WPI_TalonSRX elevationMotor;

    @Log.SpeedController(name = "Ball Propulsion Motor")
    private WPI_TalonFX ballPropulsionMotor;
    private WPI_TalonFX ballPropulsionFollower;

    @Log.SpeedController(name = "Ball Feeder Motor")
    private WPI_TalonSRX feeder;

    // Neos

    //////////////////////////////////////////////////////////////////////////////
    // Smart Dashboard Variables

    double smartDashboardGetterRateLimiterClock;

    private double feederOutputPercent;
    private double BMSOutputPercent;
    
    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        this.visionSubsystem = visionSubsystem;
        shooterCalculator = new ShooterCalculator(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        azimuthMotor = MotorUtils.makeSRX(config.shooter.azimuth);
        elevationMotor = MotorUtils.makeSRX(config.shooter.elevation);

        azimuthMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                0);
        azimuthMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                0);

        azimuthMotor.overrideLimitSwitchesEnable(true);

        elevationMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, 0);
        elevationMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, 0);

        elevationMotor.overrideLimitSwitchesEnable(true);

        ballPropulsionMotor = MotorUtils.makeFX(config.shooter.shooter);
        //
        ballPropulsionMotor.configOpenloopRamp(0);
        ballPropulsionMotor.configClosedloopRamp(0);
        ballPropulsionMotor.setNeutralMode(NeutralMode.Coast);

        ballPropulsionFollower = MotorUtils.makeFX(config.shooter.shooterFollower);

        feeder = MotorUtils.makeSRX(config.shooter.feeder);
        feeder.selectProfileSlot(MotorUtils.velocitySlot, 0);
        feeder.setNeutralMode(NeutralMode.Brake);

        ballPropulsionMotor.selectProfileSlot(MotorUtils.velocitySlot, 0);

        if (config.enableBallManagementSubsystem) {
            ballManagementSubsystem = new BallManagementSubsystem(config);
            ballManagementSubsystem.initialize();
        }

        rightAzimuthSoftLimit_ticks = MathUtils.unitConverter(config.shooter.rightAzimuthSoftLimit_deg, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;
        leftAzimuthSoftLimit_ticks = MathUtils.unitConverter(config.shooter.leftAzimuthSoftLimit_deg, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        forwardElevationSoftLimit_ticks = MathUtils.unitConverter(config.shooter.forwardElevationSoftLimit_deg, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;
        backwardElevationSoftLimit_ticks = MathUtils.unitConverter(config.shooter.backwardElevationSoftLimit_deg, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;
        // You may have noticed the usage of MathUtils.unitConverter, if you're
        // wondering what it does and how it works, then look no further!
        /**
         * The unit converter works like this: The first variable is the amount, lets
         * just say 10 for an example.
         *
         * The second is the unit to be converted, or the unit in which the amount is
         * measured. For example let's say the amount is in degrees, that would mean
         * this number is 360.
         *
         * The third is what the previous two should be converted into. For this example
         * I'll use radians, that means this number would be 6.28319.
         *
         * So in this example we have MathUtils.unitConverter(10, 360, 6.28319) which
         * would convert 10 degrees into radians.
         */

        //

        if (config.shooter.rightAzimuthSoftLimit_deg != -1 && config.shooter.leftAzimuthSoftLimit_deg != -1) {
            azimuthMotor.configForwardSoftLimitEnable(true);
            azimuthMotor.configForwardSoftLimitThreshold((int) rightAzimuthSoftLimit_ticks);

            azimuthMotor.configReverseSoftLimitEnable(true);
            azimuthMotor.configReverseSoftLimitThreshold((int) -leftAzimuthSoftLimit_ticks);
        }
        if (config.shooter.forwardElevationSoftLimit_deg != -1 && config.shooter.backwardElevationSoftLimit_deg != -1) {
            elevationMotor.configForwardSoftLimitEnable(true);
            elevationMotor.configForwardSoftLimitThreshold((int) forwardElevationSoftLimit_ticks);

            elevationMotor.configReverseSoftLimitEnable(true);
            elevationMotor.configReverseSoftLimitThreshold((int) -backwardElevationSoftLimit_ticks);
        }

        shooterCalculator.initialize(visionSubsystem);

        ballPropulsionMotor.configClosedloopRamp(0);
        ballPropulsionFollower.configClosedloopRamp(0);

        ballPropulsionMotor.enableVoltageCompensation(true);
        ballPropulsionMotor.configVoltageCompSaturation(12);
        ballPropulsionMotor.configNominalOutputReverse(0);
        ballPropulsionMotor.setNeutralMode(NeutralMode.Coast);

        ballPropulsionFollower.enableVoltageCompensation(true);
        ballPropulsionFollower.configVoltageCompSaturation(12);
        ballPropulsionFollower.configNominalOutputReverse(0);
        ballPropulsionFollower.setNeutralMode(NeutralMode.Coast);

        ballPropulsionMotor.configVelocityMeasurementWindow(1);
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void diagnosticsCheck() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic(float deltaTime) {

        calculateAbsoluteDegreesToRotate();

        // Calculate the target position of the turret in manual mode.
        targetPositionAzimuth_ticks = (int) (targetPositionAzimuth_ticks + (targetChangeAzimuth_ticks * deltaTime));
        targetPositionElevation_ticks = (int) (targetPositionElevation_ticks
                + (targetChangeElevation_ticks * deltaTime));

        // Apply the soft limits.
        if (config.shooter.rightAzimuthSoftLimit_deg != -1 && config.shooter.leftAzimuthSoftLimit_deg != -1) {
            if (targetPositionAzimuth_ticks > rightAzimuthSoftLimit_ticks) {
                targetPositionAzimuth_ticks = (int) rightAzimuthSoftLimit_ticks;
            } else if (targetPositionAzimuth_ticks < -leftAzimuthSoftLimit_ticks) {
                targetPositionAzimuth_ticks = (int) -leftAzimuthSoftLimit_ticks;
            }
        }
        if (config.shooter.forwardElevationSoftLimit_deg != -1 && config.shooter.backwardElevationSoftLimit_deg != -1) {
            if (targetPositionElevation_ticks > forwardElevationSoftLimit_ticks) {
                targetPositionElevation_ticks = (int) forwardElevationSoftLimit_ticks;
            } else if (targetPositionElevation_ticks < -backwardElevationSoftLimit_ticks) {
                targetPositionElevation_ticks = (int) -backwardElevationSoftLimit_ticks;
            }
        }
        reverseElevationLimitSwitchClosed = elevationMotor.getSensorCollection().isRevLimitSwitchClosed();
        leftAzimuthLimitSwitchClosed = azimuthMotor.getSensorCollection().isRevLimitSwitchClosed();
        rightAzimuthLimitSwitchClosed = azimuthMotor.getSensorCollection().isFwdLimitSwitchClosed();
        // Checks if the limit switch for the elevation motor is closed, if it is: set
        // the motor's sensor position to ELEVATION_LIMIT_SWITCH_DEG.
        // The reason it's a constant and not just 0 is because the limit switch could
        // be somewhere else.
        if (reverseElevationLimitSwitchClosed) {
            elevationMotor.setSelectedSensorPosition(
                    (int) (MathUtils.unitConverter(config.shooter.ELEVATION_LIMIT_SWITCH_DEG, 360,
                            config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio));
// 254
        }
        if (leftAzimuthLimitSwitchClosed) {
            azimuthMotor.setSelectedSensorPosition(
                    (int) (MathUtils.unitConverter(-config.shooter.AZIMUTH_LEFT_LIMIT_SWITCH_DEG, 360,
                            config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio));
        }
        if (rightAzimuthLimitSwitchClosed) {
            azimuthMotor.setSelectedSensorPosition(
                    (int) (MathUtils.unitConverter(config.shooter.AZIMUTH_RIGHT_LIMIT_SWITCH_DEG, 360,
                            config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio));
        }

        azimuthMotor.set(ControlMode.MotionMagic, targetPositionAzimuth_ticks);

        if (spinningUp) {
            spinUp();
            elevationMotor.set(ControlMode.MotionMagic, targetPositionElevation_ticks);
        } else {
            elevationMotor.set(0);
        }
    }

    /**
     * Spin up the shooter, then spin up the feeder when the shooter reaches it's
     * target velocity.
     */
    public void spinUp() {
        if (!autoAiming) {
            shooterVelocity_ticks = (float) MathUtils.unitConverter(config.shooter.getShooterVelocity_rpm(), 600,
                    config.shooter.shooter.ticksPerRevolution) * config.shooter.shooterGearRatio;
        }
        double averageError = feederFilter
                .calculate((double) Math.abs(ballPropulsionMotor.getSelectedSensorVelocity() - shooterVelocity_ticks));

        SmartDashboard.putNumber(getName() + "/averageError", averageError);
        // Spin up the feeder if the shooter is up to speed.
        if (averageError <= config.shooter.feederSpinUpDeadband_ticks && feeding) {
            feeder.set(feederOutputPercent);
            SmartDashboard.putString(getName() + "/Feeder State", "Feeding");
            upToSpeed = true;
        } // If it isn't though, turn the feeder off.
        else {
            upToSpeed = false;
            feeder.set(0);
            SmartDashboard.putString(getName() + "/Feeder State", "Cannot fire: Shooter hasn't been spun up!");
        }
        // Spin up the shooter.
        // If we're auto-aiming, it'll use the ticks specified by the vision subsystem.
        // Otherwise, it'll use the one set in NT
        setShooterVelocity((int) shooterVelocity_ticks);
        // ballPropulsionMotor.set(ControlMode.PercentOutput,
        // (float)SmartDashboard.getNumber(getName() + "/Shooter %Output", 0.5));
        SmartDashboard.putString(getName() + "/Shooter State", "Shooting");
    }

    public void stopSpinningUp() {
        // Stop the feeder.
        feeder.set(0);
        SmartDashboard.putString(getName() + "/Feeder State", "Doing Nothing");

        // Stop the shooter.
        ballPropulsionMotor.set(0);
        SmartDashboard.putString(getName() + "/Shooter State", "Doing Nothing");

        // Stop spinning up.
        upToSpeed = false;
        spinningUp = false;
    }

    public void spinBMS() {

        // Make sure the BMS is enabled before calling it, if you don't do this, you'll
        // get a null pointer exception.
        if (config.enableBallManagementSubsystem) {
            ballManagementSubsystem.fire((float) BMSOutputPercent);
        } // If it's not, make this clear.
        else {
            SmartDashboard.putString("BallManagementSubsystem/State",
                    "Cannot fire: BallManagementSubsystem is not enabled.");
        }
    }

    // Stop firing.
    public void holdFire() {
        if (config.enableBallManagementSubsystem) {
            ballManagementSubsystem.doNotFire();
        }
    }

    public void rotate(double azimuthInput, double elevationInput) {
        double spinRateAzimuth;
        double spinRateElevation;

        // ensure axis input is above deadband
        if (azimuthInput >= config.shooter.manualAzimuthDeadband || elevationInput >= config.shooter.manualElevationDeadband) {
            spinRateAzimuth = azimuthInput;
            spinRateElevation = elevationInput;
        } else {
            spinRateAzimuth = 0;
            spinRateElevation = 0;
        }

        // Turn turret at a quantity of degrees per second configurable in the smart
        // dashboard.
        double smartDashboardTurnRateTicksAzimuth = MathUtils.unitConverter(config.shooter.getAzimuthTurnRate_deg(), 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        double smartDashboardTurnRateTicksElevation = MathUtils.unitConverter(config.shooter.getElevationTurnRate_deg(), 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        // Target position changes by this number every time periodic is called.
        targetChangeAzimuth_ticks = (int) (smartDashboardTurnRateTicksAzimuth * spinRateAzimuth);
        targetChangeElevation_ticks = (int) (smartDashboardTurnRateTicksElevation * spinRateElevation);
    }

    public void rotateToDeg(double targetPointAzimuth, double targetPointElevation) {
        // Calculate the target position of the turret in ticks.
        double targetPointTicksAzimuth = MathUtils.unitConverter(targetPointAzimuth, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        double targetPointTicksElevation = MathUtils.unitConverter(targetPointElevation, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        // Set the target position to the calculated number.
        targetPositionAzimuth_ticks = (int) (targetPointTicksAzimuth);
        targetChangeAzimuth_ticks = 0;

        targetPositionElevation_ticks = (int) (targetPointTicksElevation);
        targetChangeElevation_ticks = 0;
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

    public void autoAimTurret() {
        var hoodAngle = shooterCalculator.calculateHoodAngle_deg();
        SmartDashboard.putNumber(getName() + "/Limelight hood angle", shooterCalculator.calculateHoodAngle_deg());
        rotateToDeg(absoluteDegreesToRotateAzimuth, hoodAngle);
    }

    // epic
    public void autoAimHoodAngle() {
    }

    public void autoAimVelocity() {
        autoAiming = true;
        shooterVelocity_ticks = shooterCalculator.calculateSpeed_ticks();
        SmartDashboard.putNumber(getName() + "/Limelight velocity", shooterVelocity_ticks);
        startSpinningUp();

        // TODO: do this stuff empirically (yay!)
        // the math rn isn' rly accurate
        // as u get closer decrease basically, as u get further, increase.
        // tho obv we'll test it and see
    }

    public void autoAim() {
        autoAimTurret();
        autoAimVelocity();
    }

    public void stopAutoAim() {
        autoAiming = false;
        stopSpinningUp();
        shooterCalculator.setTargetLocked(false);
    }

    public void calculateAbsoluteDegreesToRotate() {
        // We believed the offset and thus the degrees might change, causing the robot
        // to possibly oscillate about its target. To prevent this, take an average.
        // Didn't make a difference, so we've disabled it. But code remains in case we
        // want
        // to use it again.

        // If enabled in the constants file, calculate the average of the last values
        // passed in (up to what FILTER_LENGTH is in VisionConstants.java).
        absoluteDegreesToRotateAzimuth = visionSubsystem.getFilteredTx(getAzimuthDeg()) +
            MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                config.shooter.azimuth.ticksPerRevolution, 360) * config.shooter.azimuthGearRatio;
    }

    public double getDegreesToRotate() {
        return absoluteDegreesToRotateAzimuth;
    }

    // Cycles through the positions until it reaches one which is higher than the
    // current target, then sets the target to that position.
    public void nextPositionElevation() {
        for (int i = 0; i < positions.length; i++) {
            int selectedPositionNumber_ticks = (int) (MathUtils.unitConverter(positions[i], 360,
                    config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio);
            if (targetPositionElevation_ticks < selectedPositionNumber_ticks
                    && positionElevationSwitcherAlreadyPressed == false) {
                targetPositionElevation_ticks = (int) (selectedPositionNumber_ticks);
                positionElevationSwitcherAlreadyPressed = true;
                break;
            }
        }
    }

    // Cycles through the positions until it reaches one which is lower than current
    // the target, then sets the target to that position.
    public void lastPositionElevation() {
        for (int i = positions.length - 1; i >= 0; i--) {
            int selectedPositionNumber_ticks = (int) (MathUtils.unitConverter(positions[i], 360,
                    config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio);
            if (targetPositionElevation_ticks > selectedPositionNumber_ticks
                    && positionElevationSwitcherAlreadyPressed == false) {
                targetPositionElevation_ticks = (int) (selectedPositionNumber_ticks);
                positionElevationSwitcherAlreadyPressed = true;
                break;
            }
        }
    }

    // The buttons which trigger the above two functions aren't supposed to be held,
    // so they set a boolean to true and don't trigger again until it's false.
    // This sets that boolean to false.
    public void resetPositionElevationSwitcher() {
        positionElevationSwitcherAlreadyPressed = false;
    }

    public double getTargetAzimuthDeg() {
        return MathUtils.unitConverter(targetPositionAzimuth_ticks, 360, config.shooter.azimuth.ticksPerRevolution)
                / config.shooter.azimuthGearRatio;
    }

    public double getTargetElevationDeg() {
        return MathUtils.unitConverter(targetPositionElevation_ticks, 360, config.shooter.elevation.ticksPerRevolution)
                / config.shooter.elevationGearRatio;
    }

    public boolean isUpToSpeed() {
        return upToSpeed;
    }

    public boolean isSpinningUp() {
        return spinningUp;
    }

    public void startSpinningUp() {
        spinningUp = true;
    }

    @Override
    protected void dashboardInit() {
        super.dashboardInit();
        SmartDashboard.putNumber(getName() + "/Feeder Output Percent", config.shooter.FEEDER_OUTPUT_PERCENT);
        SmartDashboard.putNumber(getName() + "/Dashboard Elevation Target",
                config.shooter.DEFAULT_ELEVATION_TARGET_DEG);

        SmartDashboard.putNumber(getName() + "/Shooter %Output", 0.5); // TODO TEMPORARY
    }

    public void zeroElevationSensor() {
        elevationMotor.setSelectedSensorPosition(0);
    }

    public void spinFeeder() {
        feeding = true;
    }

    public void stopSpinningFeeder() {
        feeding = false;
    }

    private double lastTime = System.nanoTime();

    public void setShooterVelocity(int tp100ms) {
        double currentTime = System.nanoTime();

        SmartDashboard.putNumber(getName() + "/dt", (currentTime - lastTime) / 1000000000);
        lastTime = currentTime;

        int error = ballPropulsionMotor.getSelectedSensorVelocity() - tp100ms;

        SmartDashboard.putNumber(getName() + "/Shooter Velocity Ticks",
                ballPropulsionMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber(getName() + "/Shooter Velocity Target", tp100ms);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity Error", error);

        SmartDashboard.putNumber(getName() + "/Shooter Velocity Current RPM",
                MathUtils.unitConverter(ballPropulsionMotor.getSelectedSensorVelocity(),
                        config.shooter.shooter.ticksPerRevolution, 600) / config.shooter.shooterGearRatio);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity Target RPM",
                MathUtils.unitConverter(tp100ms, config.shooter.shooter.ticksPerRevolution, 600)
                        / config.shooter.shooterGearRatio);
        SmartDashboard.putNumber(getName() + "/Shooter Velocity Error RPM",
                MathUtils.unitConverter(error, config.shooter.shooter.ticksPerRevolution, 600)
                        / config.shooter.shooterGearRatio);

        if (config.shooter.USE_BANG_BANG) {
            if (error < -tp100ms / 2) {
                ballPropulsionMotor.set(config.shooter.BANG_BANG_RAMP_UP_PERCENT);
                // I.e if our error is -300 and our tp100ms is 300, our -tp100ms is -300.
                // divide that by 2 and you get -150.
                // -300 is less than -150, so the shooter will ramp up.

                // However if our error is 300, 300 is definitely MORE than -150,
                // so it will continue on to the other if statements.

            } else if (error < 0 && error > -tp100ms / 8) {
                ballPropulsionMotor.set(config.shooter.BANG_BANG_MAINTAIN_SPEED_PERCENT);
            } else if (error < config.shooter.BANG_BANG_ERROR) {
                ballPropulsionMotor.set(config.shooter.BANG_BANG_PERCENT);
            } else {
                ballPropulsionMotor.set(0);
            }
        } else {
            ballPropulsionMotor.set(ControlMode.Velocity, tp100ms);
        }
    }

    @Override
    public void listTalons() {
        talons.add(azimuthMotor);
        talons.add(elevationMotor);
        talons.add(ballPropulsionMotor);
        talons.add(feeder);
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // Put the outputs on the smart dashboard.
        smartDashboardGetterRateLimiterClock = smartDashboardGetterRateLimiterClock + deltaTime;
        if (getTelemetryEnabled()) {
            SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + "/Feeder Output", feeder.getMotorOutputPercent());

            SmartDashboard.putNumber(getName() + "/Shooter current", ballPropulsionMotor.getSupplyCurrent());

            SmartDashboard.putNumber(getName() + "/Azimuth Position ", azimuthMotor.getSelectedSensorPosition());

            SmartDashboard.putNumber(getName() + "/Azimuth Target Position Deg ",
                    MathUtils.unitConverter(targetPositionAzimuth_ticks, config.shooter.azimuth.ticksPerRevolution, 360)
                            * config.shooter.azimuthGearRatio);

            SmartDashboard.putNumber(getName() + "/Azimuth Position Deg ",
                    MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                            config.shooter.azimuth.ticksPerRevolution, 360) * config.shooter.azimuthGearRatio);

            SmartDashboard
                    .putNumber(getName() + "/Elevation Position Deg ",
                            MathUtils.unitConverter(elevationMotor.getSelectedSensorPosition(),
                                    config.shooter.elevation.ticksPerRevolution, 360)
                                    * config.shooter.elevationGearRatio);

            SmartDashboard.putNumber(getName() + "/Elevation Position ", elevationMotor.getSelectedSensorPosition());

            SmartDashboard
                    .putNumber(getName() + "/Elevation Target Position Deg ",
                            MathUtils.unitConverter(targetPositionElevation_ticks,
                                    config.shooter.elevation.ticksPerRevolution, 360)
                                    * config.shooter.elevationGearRatio);

            SmartDashboard.putNumber(getName() + "/Hood Angle", shooterCalculator.calculateHoodAngle_deg());

            SmartDashboard.putNumber(getName() + "/Falcon temperature",
                    (32 + 1.8 * ballPropulsionMotor.getTemperature()));

            SmartDashboard.putBoolean(getName() + "/Reverse Elevation Limit Switch Closed?",
                    reverseElevationLimitSwitchClosed);
            SmartDashboard.putBoolean(getName() + "/Left Azimuth Limit Switch Closed?", leftAzimuthLimitSwitchClosed);
            SmartDashboard.putBoolean(getName() + "/Right Azimuth Limit Switch Closed?", rightAzimuthLimitSwitchClosed);

            SmartDashboard.putNumber(getName() + "/SmartDashboard Getter Rate Limiter Clock",
                    smartDashboardGetterRateLimiterClock);
        }

        if (smartDashboardGetterRateLimiterClock >= 1) {
            smartDashboardGetterRateLimiterClock = 0;

            feederOutputPercent = SmartDashboard.getNumber(getName() + "/Feeder Output Percent",
                    config.shooter.FEEDER_OUTPUT_PERCENT);

            BMSOutputPercent = SmartDashboard.getNumber(getName() + "/BallManagementSubsystem/Output Percent",
                    config.ballManagement.BMS_OUTPUT_PERCENT);
        }
    }

    // Turns everything off immediately.
    public void disable() {
        azimuthMotor.set(0);
        elevationMotor.set(0);
        ballPropulsionMotor.set(0);
        feeder.set(0);

        upToSpeed = false;
        positionElevationSwitcherAlreadyPressed = false;

        targetPositionAzimuth_ticks = 0;
        targetChangeAzimuth_ticks = 0;
        targetPositionElevation_ticks = 0;
        targetChangeElevation_ticks = 0;
        absoluteDegreesToRotateAzimuth = 0;
    }
    @Override
    public String configureLogName() {
        return "Shooter";
    }
}
