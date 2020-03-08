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
        Extending, Retracting, Off, Winding, Rewinding;
    }

    private boolean active = false;
    private boolean rewindEnabled = false;
    protected WPI_TalonSRX motorRight;
    protected WPI_TalonSRX motorLeft;
    private ClimbState climbState = ClimbState.Off;

    // PREPARE FOR CARGO CULTED SHOOTER CODE!
    private int targetPosition_ticks;
    private int targetChange_ticks;
    private int softLimit_ticks = ClimbConstants.SOFT_LIMIT_TICKS;
    public ClimbSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        super.initialize();
        motorRight = MotorUtils.makeSRX(config.climb.climbRight);
        motorLeft = MotorUtils.makeSRX(config.climb.climbLeft);
        // Config exists for a reason.
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
        if (active || rewindEnabled) {
            switch (climbState) {
                case Off:
                    motorRight.set(0);
                    motorLeft.set(0);
                    break;

                case Extending:
                    // motorRight.set(
                    // SmartDashboard.getNumber(getName() + "/Climber Current",
                    // ClimbConstants.EXTEND_OUTPUT));
                    targetPosition_ticks = ClimbConstants.EXTEND_TICKS;
                    motorRight.set(ControlMode.MotionMagic, targetPosition_ticks);
                    motorLeft.set(ControlMode.MotionMagic, targetPosition_ticks);
                    break;

                // no holding state is needed, since a mechanical ratchet will hold the robot
                // while its hanging
                case Retracting:
                    targetChange_ticks = (int) (SmartDashboard.getNumber(getName() + "/Retract Rate",
                            ClimbConstants.RETRACT_RATE_TICKS) * deltaTime);
                    targetPosition_ticks = targetPosition_ticks + targetChange_ticks;
                    targetChange_ticks = 0;
                    // motorRight.set(
                    // SmartDashboard.getNumber(getName() + "/Climber Current",
                    // ClimbConstants.RETRACT_OUTPUT));
                    if (targetPosition_ticks > softLimit_ticks) {
                        targetPosition_ticks = softLimit_ticks;
                    }
                    motorRight.set(ControlMode.MotionMagic, targetPosition_ticks);
                    motorLeft.set(ControlMode.MotionMagic, targetPosition_ticks);
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

    public void activateClimb() {
        active = true;
    }

    public boolean isExtending() {
        return climbState == ClimbState.Extending;
    }

    public boolean isRewindEnabled(){
        return rewindEnabled;
    }

    public void off() {
        climbState = ClimbState.Off;
    }

    public void retracting() {
        if (active) {
            climbState = ClimbState.Retracting;
        }
    }

    public void extending() {
        if (active) {
            climbState = ClimbState.Extending;
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
        SmartDashboard.putNumber(getName() + "/Right Selected Sensor Position", motorRight.getSelectedSensorPosition());
        SmartDashboard.putNumber(getName() + "/Left Selected Sensor Position", motorLeft.getSelectedSensorPosition());
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