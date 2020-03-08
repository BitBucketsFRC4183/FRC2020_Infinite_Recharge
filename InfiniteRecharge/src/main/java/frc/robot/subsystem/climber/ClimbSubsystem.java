package frc.robot.subsystem.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.talonutils.MotorUtils;

public class ClimbSubsystem extends BitBucketSubsystem {
    public enum ClimbState {
        Off, Winding, Rewinding;
    }

    private boolean active = false;
    private boolean rewindEnabled = false;
    protected WPI_TalonSRX motorRight;
    protected WPI_TalonSRX motorLeft;
    private ClimbState climbState = ClimbState.Off;

    public ClimbSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        motorRight = MotorUtils.makeSRX(config.climb.climbRight);
        motorLeft = MotorUtils.makeSRX(config.climb.climbLeft);
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        rewindEnabled = SmartDashboard.getBoolean(getName() + "/Rewind Enabled", false);
        if (rewindEnabled) {
            switch (climbState) {
                case Off:
                    motorRight.set(0);
                    motorLeft.set(0);
                    break;
                case Winding:
                    motorLeft.set(-SmartDashboard.getNumber(getName() + "/Climber Wind", ClimbConstants.REWIND_OUTPUT));
                    motorRight.set(-SmartDashboard.getNumber(getName() + "/Climber Wind", ClimbConstants.REWIND_OUTPUT));
                    break;
                case Rewinding:
                    motorLeft.set(SmartDashboard.getNumber(getName() + "/Climber Wind", ClimbConstants.REWIND_OUTPUT));
                    motorRight.set(SmartDashboard.getNumber(getName() + "/Climber Wind", ClimbConstants.REWIND_OUTPUT));
                    break;
            }
        }
    }

    public void toggleActive() {
        active = !active;
    }

    public boolean isRewindEnabled(){
        return rewindEnabled;
    }

    public boolean isActive(){
        return active;
    }

    public void off() {
        climbState = ClimbState.Off;
    }

    public void manualClimb(double leftStick, double rightStick){
        leftStick = Math.abs(leftStick);
        rightStick = Math.abs(rightStick);

        if (leftStick > ClimbConstants.DEADBAND) {
            motorLeft.set(ControlMode.PercentOutput, leftStick * ClimbConstants.OUTPUT);
        } else {
            motorLeft.set(ControlMode.PercentOutput, 0);
        }
        if (rightStick > ClimbConstants.DEADBAND) {
            motorRight.set(ControlMode.PercentOutput, rightStick * ClimbConstants.OUTPUT);
        } else {
            motorRight.set(ControlMode.PercentOutput, 0);
        }
    }

    public void rewinding() {
        if (rewindEnabled) {
            climbState = ClimbState.Rewinding;
        }
    }

    public void winding() {
        if (rewindEnabled) {
            climbState = ClimbState.Winding;
        }
    }

    @Override
    public void dashboardInit() {
        super.dashboardInit();
        SmartDashboard.putBoolean(getName() + "/Rewind Enabled", false);
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        SmartDashboard.putNumber(getName() + "/Right Motor Output", motorRight.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "/Left Motor Output", motorLeft.getMotorOutputPercent());
        SmartDashboard.putBoolean(getName() + "/Active", active);
        SmartDashboard.putString(getName() + "/Climb State", climbState.toString());
    }

    public void disable() {
        motorLeft.set(0);
        motorRight.set(0);
    }

    @Override
    public void listTalons() {
        talons.add(motorRight);
        talons.add(motorLeft);
    }
}