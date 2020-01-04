package frc.robot.utils.control.motor;



import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.encoder.QuadratureEncoder;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;

import frc.robot.utils.math.units.BaseUnit;
import frc.robot.utils.math.units.Units;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;



/**
 * Wrapper class for CANSparkMax motors
 */
public class BBSparkMax extends BBMotorController {
    private final CANSparkMax MOTOR;
    private final CANPIDController PID_CONTROLLER;

    private CANEncoder encoder;

    private int slotInUse = 0;



    public BBSparkMax(int deviceID, MotorType type) {
        super(deviceID);

        MOTOR = new CANSparkMax(deviceID, type);

        PID_CONTROLLER = MOTOR.getPIDController();
    }



    @Override
    protected void loadPID(PID constants, int slot) {
        PID_CONTROLLER.setP(constants.getKP(), slot);
        PID_CONTROLLER.setI(constants.getKI(), slot);
        PID_CONTROLLER.setD(constants.getKD(), slot);
        PID_CONTROLLER.setIZone(constants.getIZone(), slot);
    }

    @Override
    protected void loadPIDF(PIDF constants, int slot) {
        loadPID(constants, slot);

        PID_CONTROLLER.setFF(constants.getKF(), slot);
    }

    @Override
    protected void clearPIDF(int slot) {
        // literally no idea how you'd here get considering there's FOUR slots...

        PID_CONTROLLER.setP(0, slot);
        PID_CONTROLLER.setI(0, slot);
        PID_CONTROLLER.setD(0, slot);
        PID_CONTROLLER.setFF(0, slot);
        PID_CONTROLLER.setIZone(0, slot);
    }



    @Override
    public void selectMotionConfigSlot(int slot) {
        this.slotInUse = slot;
    }

    @Override
    protected int getMaxMotionSlots() {
        return 4;
    }



    @Override
    public void cmdPosition_native(double val_nu, ControlType controlMethod) {
        // RevRobotics also has a ControlType class :/
        com.revrobotics.ControlType mode;

        switch (controlMethod) {
            case Position: {
                mode = com.revrobotics.ControlType.kPosition;
                break;
            }
            case MotionMagic: {
                mode = com.revrobotics.ControlType.kSmartMotion;
                break;
            }
            default: {
                return;
            }
        }

        PID_CONTROLLER.setReference(val_nu, mode, slotInUse);
    }

    @Override
    protected void cmdVelocity_native(double vel) {
        PID_CONTROLLER.setReference(vel, com.revrobotics.ControlType.kVelocity, slotInUse);
    }

    @Override
    public void cmdPercent_native(double perc) {
        // percent control under PID because RevRobotics API kinda quirky
        PID_CONTROLLER.setReference(perc, com.revrobotics.ControlType.kDutyCycle, slotInUse);
    }

    @Override
    protected void loadMotionMagic(double acc, double vel, int slot) {
        PID_CONTROLLER.setSmartMotionMaxAccel(acc, slot);
        PID_CONTROLLER.setSmartMotionMaxVelocity(vel, slot);
    }

    @Override
    protected void clearMotionMagic(int slot) {
        PID_CONTROLLER.setSmartMotionMaxAccel(0, slot);
        PID_CONTROLLER.setSmartMotionMaxVelocity(0, slot);
    }

    @Override
    protected void addQuadratureEncoder(QuadratureEncoder sensor) {
        encoder = MOTOR.getEncoder(EncoderType.kQuadrature, sensor.getCPR());
        encoder.setPosition(0);
    }

    @Override
    public double getPosition_nu() {
        // SparkMax returns position in revs
        return encoder.getPosition();
    }



    @Override
    protected BaseUnit getThetaUnit_nu() {
        return Units.REV;
    }
    @Override
    protected BaseUnit getTimeUnit_nu() {
        return Units.MIN; // RPM -> minutes -> 60 seconds
    }

    @Override
    protected BaseUnit getSecondTimeUnit_nu() {
        return Units.S;
    }



    @Override
    public double getVelocity_nu() {
        return encoder.getVelocity(); // I'm somebody
    }



    @Override
    public double getVoltage() {
        return MOTOR.getBusVoltage();
    }

    @Override
    public double getPercentVoltage() {
        return getVoltage() / RobotController.getBatteryVoltage();
    }



    @Override
    public void follow(BBMotorController motorController) {
        if (motorController instanceof BBSparkMax) {
            CANSparkMax leader = ((BBSparkMax) motorController).getSparkMax();

            MOTOR.follow(leader);
        }
    }



    @Override
    public void setInverted(boolean invert) {
        setInverted(invert);
    }

    @Override
    public void setSensorPhase(boolean phase) {
        if (encoder != null) {
            encoder.setInverted(phase);
        }
    }



    @Override
    public void setOpenLoopRampRate(double fullThrottleSec) {
        MOTOR.setOpenLoopRampRate(fullThrottleSec);
    }

    @Override
    public void setClosedLoopRampRate(double fullThrottleSec) {
        MOTOR.setClosedLoopRampRate(fullThrottleSec);
    }

    @Override
    public double getCurrent() {
        return MOTOR.getOutputCurrent();
    }

    @Override
    protected void setPosition_nu(double pos_nu) {
        encoder.setPosition(pos_nu);
    }



    public CANSparkMax getSparkMax() { return MOTOR; }
}