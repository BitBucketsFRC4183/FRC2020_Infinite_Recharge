package frc.robot.utils.talonutils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.config.MotorConfig;

public class MotorUtils {

    static final int positionSlot = 0; // ALWAYS! Don't EVER set it to anything else.
    public static final int velocitySlot = 1; // ALWAYS! Don't EVER set it to anything else.

    public static int MAX_STATUS_FRAME_PERIOD = 160;

    /**
     * initializeMotor - set all of the motor configuration states to a known value
     * This is important when we are not sure if the motor is in a factory state
     * 
     * @param motor
     *
     *              The following T O D O is done. T O D O: Move this to a separate
     *              package?
     */
    public static void initializeMotorDefaults(WPI_TalonSRX motor) {
        // TODO: Check ErrorCode?
        motor.configFactoryDefault(RobotMap.CONTROLLER_TIMEOUT_MS);

        motor.stopMotor();

    }

    public static void initializeMotorFPID(WPI_TalonSRX motor, double kF, double kP, double kI, double kD, int iZone) {
        /**
         * The following T O D O is done. T O D O: Actually write this function.
         */

        int slotIdx = 0;
        initializeMotorFPID(motor, kF, kP, kI, kD, iZone, slotIdx);
    }

    public static void initializeMotorFPID(WPI_TalonSRX motor, double kF, double kP, double kI, double kD, int iZone,
            int slotIdx) {
        /**
         * The following T O D O is done. T O D O: Actually write this function too.
         */

        int timeout = RobotMap.CONTROLLER_TIMEOUT_MS;

        motor.selectProfileSlot(slotIdx, RobotMap.PRIMARY_PID_LOOP);
        motor.config_kF(slotIdx, kF, timeout);
        motor.config_kP(slotIdx, kP, timeout);
        motor.config_kI(slotIdx, kI, timeout);
        motor.config_kD(slotIdx, kD, timeout);
        motor.config_IntegralZone(slotIdx, iZone, RobotMap.CONTROLLER_TIMEOUT_MS);
    }

    public static void initializeQuadEncoderMotor(WPI_TalonSRX motor) {
        initializeQuadEncoderMotor(motor, MAX_STATUS_FRAME_PERIOD);
    }

    public static void initializeMagEncoderRelativeMotor(WPI_TalonSRX motor) {
        initializeMagEncoderRelativeMotor(motor, MAX_STATUS_FRAME_PERIOD);
    }

    public static void initializeMagEncoderAbsoluteMotor(WPI_TalonSRX motor) {
        initializeMagEncoderAbsoluteMotor(motor, MAX_STATUS_FRAME_PERIOD);
    }

    /**
     * Initializes the quad encoder motor, whatever that means.
     */
    public static void initializeQuadEncoderMotor(WPI_TalonSRX motor, int statusFramePeriod) {
        initializeQuadEncoderMotor(motor, statusFramePeriod, RobotMap.PRIMARY_PID_LOOP);

    }

    /**
     * Initializes the quad encoder motor, whatever that means.
     */
    public static void initializeQuadEncoderMotor(WPI_TalonSRX motor, int statusFramePeriod, int pidLoop) {
        int timeout = RobotMap.CONTROLLER_TIMEOUT_MS;

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidLoop, timeout);
        motor.setSelectedSensorPosition(0, pidLoop, timeout);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFramePeriod, timeout);

    }

    /**
     * Initializes the motor controller to have a relative mag encoder.
     */
    public static void initializeMagEncoderRelativeMotor(WPI_TalonSRX motor, int statusFramePeriod) {
        int timeout = RobotMap.CONTROLLER_TIMEOUT_MS;
        int pidLoop = RobotMap.PRIMARY_PID_LOOP;

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidLoop, timeout);
        motor.setSelectedSensorPosition(0, pidLoop, timeout);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFramePeriod, timeout);

    }

    /**
     * Initializes the motor controller to have a relative mag encoder.
     */
    public static void initializeMagEncoderAbsoluteMotor(WPI_TalonSRX motor, int statusFramePeriod) {
        int timeout = RobotMap.CONTROLLER_TIMEOUT_MS;
        int pidLoop = RobotMap.PRIMARY_PID_LOOP;

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, pidLoop, timeout);
        motor.setSelectedSensorPosition(0, pidLoop, timeout);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFramePeriod, timeout);

    }

    public static void motorInit(BaseTalon motor, MotorConfig motorConfig) {
        motor.configFactoryDefault();

        motor.config_kF(positionSlot, motorConfig.positionPIDF.getKF());
        motor.config_kP(positionSlot, motorConfig.positionPIDF.getKP());
        motor.config_kI(positionSlot, motorConfig.positionPIDF.getKI());
        motor.config_kD(positionSlot, motorConfig.positionPIDF.getKD());
        motor.config_IntegralZone(positionSlot, (int) motorConfig.positionPIDF.getIZone());

        motor.config_kF(velocitySlot, motorConfig.velocityPIDF.getKF());
        motor.config_kP(velocitySlot, motorConfig.velocityPIDF.getKP());
        motor.config_kI(velocitySlot, motorConfig.velocityPIDF.getKI());
        motor.config_kD(velocitySlot, motorConfig.velocityPIDF.getKD());
        motor.config_IntegralZone(velocitySlot, (int) motorConfig.velocityPIDF.getIZone());
        motor.setSensorPhase(motorConfig.sensorPhase);
        motor.setInverted(motorConfig.inverted);

        switch (motorConfig.encoderType) {
        case None:
            motor.configSelectedFeedbackSensor(FeedbackDevice.None);
            break;

        case Quadrature:
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            motor.setSelectedSensorPosition(0);
            break;

        case Relative:
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            motor.setSelectedSensorPosition(0);
            break;

        case Absolute:
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
            break;

        case Integrated:
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            motor.setSelectedSensorPosition(0);
            break;
        }

        motor.configMotionCruiseVelocity(motorConfig.motionMagicCruiseVelocity);
        motor.configMotionAcceleration(motorConfig.motionMagicAcceleration);

        if (motorConfig.followingID != -1) {
            motor.set(ControlMode.Follower, motorConfig.followingID);
        }

    }

    public static void motorInit(CANSparkMax motor, MotorConfig motorConfig) {
        /* Configure Sensor Source for velocity PID */
        CANEncoder encoder;

        // brushless motors can't be inverted
        // encoder.setInverted(settings.sensorPhase);

        /* Set acceleration and vcruise velocity - see documentation */
        CANPIDController pidController = motor.getPIDController();

        // configure position PID constants
        pidController.setFF(motorConfig.positionPIDF.getKF(), positionSlot);
        pidController.setP(motorConfig.positionPIDF.getKP(), positionSlot);
        pidController.setD(motorConfig.positionPIDF.getKD(), positionSlot);
        pidController.setI(motorConfig.positionPIDF.getKI(), positionSlot);
        pidController.setIZone(motorConfig.positionPIDF.getIZone(), positionSlot);

        // configure velocity PID constants
        pidController.setFF(motorConfig.velocityPIDF.getKF(), velocitySlot);
        pidController.setP(motorConfig.velocityPIDF.getKP(), velocitySlot);
        pidController.setI(motorConfig.velocityPIDF.getKD(), velocitySlot);
        pidController.setD(motorConfig.velocityPIDF.getKI(), velocitySlot);
        pidController.setIZone(motorConfig.velocityPIDF.getIZone(), velocitySlot);
        motor.setInverted(motorConfig.inverted);
        switch (motorConfig.encoderType) {
        case Quadrature:
            encoder = motor.getEncoder(EncoderType.kQuadrature, 8192);
            break;

        case Relative:
            encoder = motor.getEncoder(EncoderType.kQuadrature, 8192);
            break;

        case Absolute:
            throw new RuntimeException("You used an absolute encoder type for a CAN Spark! You ABSOLUTE fool!");

        case Integrated:
            encoder = motor.getEncoder();
            break;
        case None:
        default:
            encoder = motor.getEncoder(EncoderType.kNoSensor, 8192);
            break;

        }
        /* Zero the sensor */
        encoder.setPosition(0);
    }

    public static WPI_TalonSRX makeSRX(MotorConfig motorConfig) {
        WPI_TalonSRX motor = new WPI_TalonSRX(motorConfig.id);
        motorInit(motor, motorConfig);
        return motor;
    }

    public static WPI_TalonFX makeFX(MotorConfig motorConfig) {
        WPI_TalonFX motor = new WPI_TalonFX(motorConfig.id);
        motorInit(motor, motorConfig);
        return motor;
    }

    public static CANSparkMax makeSpark(MotorConfig motorConfig) {
        CANSparkMax motor = new CANSparkMax(motorConfig.id, MotorType.kBrushless);
        motorInit(motor, motorConfig);
        return motor;
    }
}
