package frc.robot.utils.control.motor;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.control.encoder.QuadratureEncoder;

import frc.robot.utils.math.units.Units;
import frc.robot.utils.math.units.BaseUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;


public class BBTalonSRX extends BBMotorController {
    private final WPI_TalonSRX MOTOR;



    public BBTalonSRX(int deviceID) {
        super(deviceID);

        MOTOR = new WPI_TalonSRX(deviceID);
    }



    @Override
    protected void loadPID(PID constants, int slot) {
        MOTOR.config_kP(slot, constants.getKP());
        MOTOR.config_kI(slot, constants.getKI());
        MOTOR.config_kD(slot, constants.getKD());
        MOTOR.config_IntegralZone(slot, (int) constants.getIZone());
    }

    @Override
    protected void loadPIDF(PIDF constants, int slot) {
        loadPID(constants, slot);

        MOTOR.config_kF(slot, constants.getKD());
    }

    @Override
    protected void clearPIDF(int slot) {
        MOTOR.config_kP(slot, 0);
        MOTOR.config_kI(slot, 0);
        MOTOR.config_kD(slot, 0);
        MOTOR.config_kF(slot, 0);
        MOTOR.config_IntegralZone(slot, 0);
    }

    @Override
    protected int getMaxMotionSlots() { return 2; }



    
    @Override
    public void selectMotionConfigSlot(int slot) {
        MOTOR.selectProfileSlot(slot, 0);
    }



    @Override
    public void cmdPosition_native(double val_nu, ControlType controlMethod) {
        ControlMode mode;

        int ticks = (int) Math.round(val_nu);

        switch (controlMethod) {
            case Position: {
                mode = ControlMode.Position;
                break;
            }
            case MotionMagic: {
                mode = ControlMode.MotionMagic;
                break;
            }
            default: {
                return;
            }
        }

        MOTOR.set(mode, ticks);
    }

    @Override
    protected void cmdVelocity_native(double vel) {
        MOTOR.set(ControlMode.Velocity, (int) vel);
    }

    @Override
    public void cmdPercent_native(double perc) {
        MOTOR.set(ControlMode.PercentOutput, perc);
    }

    @Override
    protected void loadMotionMagic(double acc, double vel, int slot) {
        int accInt = (int) Math.round(acc);
        int velInt = (int) Math.round(vel);

        MOTOR.configMotionAcceleration(accInt);
        MOTOR.configMotionCruiseVelocity(velInt);
    }

    @Override
    protected void clearMotionMagic(int slot) {
        // could set to 0, but TalonSRX actually doesn't store MM per slot
        // and instead stores for the motor controller, so no need to
        // (will never need to get wiped)
    }

    @Override
    protected void addQuadratureEncoder(QuadratureEncoder sensor) {
        MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        MOTOR.setSelectedSensorPosition(0);
        MOTOR.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100);
    }

    @Override
    public double getPosition_nu() {
        return MOTOR.getSelectedSensorPosition();
    }



    @Override
    protected BaseUnit getTimeUnit_nu() {
        return Units.MS100; // ticksPer100ms -> 100ms
    }

    @Override
    protected BaseUnit getSecondTimeUnit_nu() {
        return Units.S; // ticksPer100msPerSec -> 1s (ik its dumb that's CTRE tho)
    }

    @Override
    protected BaseUnit getThetaUnit_nu() {
        if (sensor == null) {
            return null;
        }

        return new BaseUnit(Units.REV, sensor.getTicksPerRev(), "tick");
    }



    @Override
    public double getVelocity_nu() {
        return MOTOR.getSelectedSensorVelocity(); // thank you CTRE for not using revs per min unlike SOMEBODY
    }



    @Override
    public double getVoltage() {
        return MOTOR.getMotorOutputVoltage();
    }

    @Override
    public double getPercentVoltage() {
        // TODO: verify this = getVoltage() / RobotController.getBatteryVoltage()
        return MOTOR.getMotorOutputPercent();
    }



    @Override
    public void follow(BBMotorController motorController) {
        if (motorController instanceof BBTalonSRX) {
            WPI_TalonSRX leader = ((BBTalonSRX) motorController).getTalonSRX();

            MOTOR.follow(leader);
        }
    }



    @Override
    public void setInverted(boolean invert) {
        MOTOR.setInverted(invert);
    }

    @Override
    public void setSensorPhase(boolean phase) {
        MOTOR.setSensorPhase(phase);
    }



    @Override
    public void setOpenLoopRampRate(double fullThrottleSec) {
        MOTOR.configOpenloopRamp(fullThrottleSec);
    }

    @Override
    public void setClosedLoopRampRate(double fullThrottleSec) {
        MOTOR.configClosedloopRamp(fullThrottleSec);
    }

    @Override
    public double getCurrent() {
        return MOTOR.getOutputCurrent();
    }

    @Override
    protected void setPosition_nu(double pos_nu) {
        MOTOR.setSelectedSensorPosition((int) Math.round(pos_nu));
    }




    public WPI_TalonSRX getTalonSRX() { return MOTOR; }
}