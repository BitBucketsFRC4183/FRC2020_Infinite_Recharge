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

import frc.robot.utils.data.filters.RunningAverageFilter;
import frc.robot.subsystem.scoring.shooter.ShooterConstants;
import frc.robot.subsystem.scoring.shooter.ball_management.BallManagementConstants;
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

    // Integers
    private int targetPositionAzimuth_ticks;
    private int targetChangeAzimuth_ticks;

    private int targetPositionElevation_ticks;
    private int targetChangeElevation_ticks;

    // Floats
    // TODO float rootBeer = good
    private float[] positions = config.shooter.elevationPositions_deg;

    // Doubles
    private double absoluteDegreesToRotateAzimuth = 0.0;
    private double shooterVelocity_ticks = 0.0;

    private double rightAzimuthSoftLimit_ticks;
    private double leftAzimuthSoftLimit_ticks;

    private double forwardElevationSoftLimit_ticks;
    private double backwardElevationSoftLimit_ticks;

    // Class Declarations
    RunningAverageFilter azimuthFilter = new RunningAverageFilter(ShooterConstants.FILTER_LENGTH);
    RunningAverageFilter elevationFilter = new RunningAverageFilter(ShooterConstants.FILTER_LENGTH);
    RunningAverageFilter feederFilter = new RunningAverageFilter(ShooterConstants.FEEDER_FILTER_LENGTH);

    public BallManagementSubsystem ballManagementSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterCalculator shooterCalculator = new ShooterCalculator();

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    private WPI_TalonSRX azimuthMotor;
    private WPI_TalonSRX elevationMotor;
    private WPI_TalonFX ballPropulsionMotor;
    private WPI_TalonFX ballPropulsionFollower;
    private WPI_TalonSRX feeder;

    // Neos

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        azimuthMotor = MotorUtils.makeSRX(config.shooter.azimuth);
        elevationMotor = MotorUtils.makeSRX(config.shooter.elevation);

        elevationMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
        elevationMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		
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
        ballPropulsionMotor.configVoltageCompSaturation(6.5);

        ballPropulsionFollower.enableVoltageCompensation(true);
        ballPropulsionFollower.configVoltageCompSaturation(6.5);

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

        targetPositionAzimuth_ticks = (int) (targetPositionAzimuth_ticks + (targetChangeAzimuth_ticks * deltaTime));
        targetPositionElevation_ticks = (int) (targetPositionElevation_ticks
                + (targetChangeElevation_ticks * deltaTime));

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
            SmartDashboard.putBoolean(getName() + "/Reverse Limit Switch Closed?", elevationMotor.getSensorCollection().isRevLimitSwitchClosed());
        if (elevationMotor.getSensorCollection().isRevLimitSwitchClosed()){
            elevationMotor.setSelectedSensorPosition(
                    (int) (MathUtils.unitConverter(ShooterConstants.ELEVATION_LIMIT_SWITCH_DEG, 360,
                            config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio));
        }

        azimuthMotor.set(ControlMode.MotionMagic, targetPositionAzimuth_ticks);
        

        if (spinningUp){
            spinUp();
            elevationMotor.set(ControlMode.MotionMagic, targetPositionElevation_ticks);
        } else {
            elevationMotor.set(0);
        }
    }

    public void spinUp() {
        if (!autoAiming) {
            shooterVelocity_ticks = (float) MathUtils
                .unitConverter(
                        SmartDashboard.getNumber(getName() + "/Shooter Velocity RPM",
                                ShooterConstants.DEFAULT_SHOOTER_VELOCITY_RPM),
                        600, config.shooter.shooter.ticksPerRevolution)
                * config.shooter.shooterGearRatio;
        }
        double averageError = feederFilter.calculate((double) Math.abs(ballPropulsionMotor.getSelectedSensorVelocity() - shooterVelocity_ticks));
        // Spin up the feeder.
        if (averageError <= config.shooter.feederSpinUpDeadband_ticks) {
            feeder.set(SmartDashboard.getNumber(getName() + "/Feeder Output Percent",
                    ShooterConstants.FEEDER_OUTPUT_PERCENT));
            SmartDashboard.putString(getName() + "/Feeder State", "Feeding");
            upToSpeed = true;
        } else {
            upToSpeed = false;
            feeder.set(0);
            SmartDashboard.putString(getName() + "/Feeder State", "Cannot fire: Shooter hasn't been spun up!");
        }
        // Spin up the shooter.
        // If we're auto-aiming, it'll use the ticks specified by the vision subsystem.
        // Otherwise, it'll use the one set in NT
        setShooterVelocity((int) shooterVelocity_ticks);
        // ballPropulsionMotor.set(ControlMode.PercentOutput, (float)SmartDashboard.getNumber(getName() + "/Shooter %Output", 0.5));
        SmartDashboard.putString(getName() + "/Shooter State", "Shooting");
    }

    public void stopSpinningUp() {
        // Spin up the feeder.
        feeder.set(0);
        SmartDashboard.putString(getName() + "/Feeder State", "Doing Nothing");

        // Spin up the shooter.
        ballPropulsionMotor.set(0);
        SmartDashboard.putString(getName() + "/Shooter State", "Doing Nothing");

        upToSpeed = false;
        spinningUp = false;
    }

    public void spinBMS() {
        if (config.enableBallManagementSubsystem) {
            ballManagementSubsystem
                    .fire((float) SmartDashboard.getNumber(getName() + "/BallManagementSubsystem/Output Percent",
                            BallManagementConstants.BMS_OUTPUT_PERCENT));
        } else {
            SmartDashboard.putString("BallManagementSubsystem/State",
                    "Cannot fire: BallManagementSubsystem is not enabled.");
        }
    }

    public void holdFire() {
        if (config.enableBallManagementSubsystem) {
            ballManagementSubsystem.doNotFire();
        }
    }

    public void rotate(double spinRateAzimuth, double spinRateElevation) {
        // Turn turret at a quantity of degrees per second configurable in the smart
        // dashboard.
        double smartDashboardTurnRateTicksAzimuth = MathUtils
                .unitConverter(
                        SmartDashboard.getNumber(getName() + "/Azimuth Turn Rate",
                                config.shooter.defaultAzimuthTurnVelocity_deg),
                        360, config.shooter.azimuth.ticksPerRevolution)
                / config.shooter.azimuthGearRatio;

        double smartDashboardTurnRateTicksElevation = MathUtils.unitConverter(
                SmartDashboard.getNumber(getName() + "/Elevation Turn Rate",
                        config.shooter.defaultElevationTurnVelocity_deg),
                360, config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

        // Target position changes by this number every time periodic is called.
        targetChangeAzimuth_ticks = (int) (smartDashboardTurnRateTicksAzimuth * spinRateAzimuth);
        targetChangeElevation_ticks = (int) (smartDashboardTurnRateTicksElevation * spinRateElevation);
    }

    public void rotateToDeg(double targetPointAzimuth, double targetPointElevation) {
        double targetPointTicksAzimuth = MathUtils.unitConverter(targetPointAzimuth, 360,
                config.shooter.azimuth.ticksPerRevolution) / config.shooter.azimuthGearRatio;

        double targetPointTicksElevation = MathUtils.unitConverter(targetPointElevation, 360,
                config.shooter.elevation.ticksPerRevolution) / config.shooter.elevationGearRatio;

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

    public void autoAimAzimuth() {
        rotateToDeg(absoluteDegreesToRotateAzimuth, getElevationDeg());
    }

    public void autoAimHoodAngle() {
        rotateToDeg(getAzimuthDeg(), shooterCalculator.calculateHoodAngle_deg());
    }

    public void autoAimVelocity() {
        autoAiming = true;
        shooterVelocity_ticks = shooterCalculator.calculateSpeed_ticks();
        startSpinningUp();

        // TODO: do this stuff empirically (yay!)
        // the math rn isn' rly accurate
        // as u get closer decrease basically, as u get further, increase.
        // tho obv we'll test it and see
    }

    public void autoAim() {
        // autoAimAzimuth();
        autoAimHoodAngle();
        autoAimVelocity();
        visionSubsystem.turnOnLEDs();
    }
    public void stopAutoAim() {
        visionSubsystem.turnOffLEDs();
        autoAiming = false;
        stopSpinningUp();
    }

    public void calculateAbsoluteDegreesToRotate() {
        boolean validTarget = visionSubsystem.getValidTarget();
        if (validTarget) {
            double tx = visionSubsystem.getTx();
            double degrees = getTargetAzimuthDegGivenOffset(tx);

            // We believed the offset and thus the degrees might change, causing the robot
            // to possibly oscillate about its target. To prevent this, take an average.
            // Didn't make a difference, so we've disabled it. But code remains in case we
            // want
            // to use it again.

            // If enabled in the constants file, calculate the average of the last values
            // passed in (up to what FILTER_LENGTH is in ShooterConstants.java).
            absoluteDegreesToRotateAzimuth = ShooterConstants.USE_AZIMUTH_FILTER ? azimuthFilter.calculate(degrees) : degrees;
        }
    }

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

    public boolean isSpinningUp(){
        return spinningUp;
    }

    public void startSpinningUp(){
        spinningUp = true;
    }

    @Override
    protected void dashboardInit() {
        super.dashboardInit();
        SmartDashboard.putNumber(getName() + "/Shooter Velocity RPM", ShooterConstants.DEFAULT_SHOOTER_VELOCITY_RPM);
        SmartDashboard.putNumber(getName() + "/Feeder Output Percent", ShooterConstants.FEEDER_OUTPUT_PERCENT);
        SmartDashboard.putNumber(getName() + "/Azimuth Turn Rate", config.shooter.defaultAzimuthTurnVelocity_deg);
        SmartDashboard.putNumber(getName() + "/Elevation Turn Rate", config.shooter.defaultAzimuthTurnVelocity_deg);
        SmartDashboard.putNumber(getName() + "/Dashboard Elevation Target", ShooterConstants.DEFAULT_ELEVATION_TARGET_DEG);

        SmartDashboard.putNumber(getName() + "/Shooter %Output", 0.5); // TODO TEMPORARY

    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // Put the outputs on the smart dashboard.
        SmartDashboard.putNumber(getName() + "/Shooter Output", ballPropulsionMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Feeder Output", feeder.getMotorOutputPercent());

        SmartDashboard.putNumber(getName() + "/Shooter current", ballPropulsionMotor.getSupplyCurrent());

        SmartDashboard.putNumber(getName() + "/Target Position ", targetPositionAzimuth_ticks);
        SmartDashboard.putNumber(getName() + "/Absolute Degrees to Rotate", absoluteDegreesToRotateAzimuth);
        SmartDashboard.putNumber(getName() + "/Azimuth Position ", azimuthMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber(getName() + "/Azimuth Target Position Deg ",
                MathUtils.unitConverter(targetPositionAzimuth_ticks, config.shooter.azimuth.ticksPerRevolution, 360)
                        * config.shooter.azimuthGearRatio);

        SmartDashboard.putNumber(getName() + "/Azimuth Position Deg ",
                MathUtils.unitConverter(azimuthMotor.getSelectedSensorPosition(),
                        config.shooter.azimuth.ticksPerRevolution, 360) * config.shooter.azimuthGearRatio);

        SmartDashboard.putNumber(getName() + "/Elevation Position Deg ",
                MathUtils.unitConverter(elevationMotor.getSelectedSensorPosition(),
                        config.shooter.elevation.ticksPerRevolution, 360) * config.shooter.elevationGearRatio);

        SmartDashboard.putNumber(getName() + "/Elevation Target Position ", targetPositionElevation_ticks);
        SmartDashboard.putNumber(getName() + "/Elevation Position ", elevationMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber(getName() + "/Elevation Target Position Deg ",
                MathUtils.unitConverter(targetPositionElevation_ticks, config.shooter.elevation.ticksPerRevolution, 360)
                        * config.shooter.elevationGearRatio);

        SmartDashboard.putNumber(getName() + "/Hood Angle", shooterCalculator.calculateHoodAngle_deg());
        
        SmartDashboard.putNumber(getName() + "/Falcon temperature", (32 + 1.8*ballPropulsionMotor.getTemperature()));
    }
    public void disable(){
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

    public void zeroElevationSensor(){
        elevationMotor.setSelectedSensorPosition(0);
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

        if (ShooterConstants.USE_BANG_BANG) {
            if (error < -tp100ms / 2) {
                ballPropulsionMotor.set(ControlMode.PercentOutput, ShooterConstants.BANG_BANG_RAMP_UP_PERCENT);
                // I.e if our error is -300 and our tp100ms is 300, our -tp100ms is -300.
                // divide that by 2 and you get -150.
                // -300 is less than -150, so the shooter will ramp up.

                // However if our error is 300, 300 is definitely MORE than -150,
                // so it will continue on to the other if statements.

            } else if (error < 0 && error > -tp100ms / 8) {
                ballPropulsionMotor.set(ControlMode.PercentOutput, ShooterConstants.BANG_BANG_MAINTAIN_SPEED_PERCENT);
            } else if (error < ShooterConstants.BANG_BANG_ERROR) {
                ballPropulsionMotor.set(ControlMode.PercentOutput, ShooterConstants.BANG_BANG_PERCENT);
            } else {
                ballPropulsionMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            ballPropulsionMotor.set(ControlMode.Velocity, tp100ms);
        }
    }

    @Override
    protected void listTalons() {
        talons.add(azimuthMotor);
        talons.add(elevationMotor);
        talons.add(ballPropulsionMotor);
        talons.add(feeder);
    }
}
