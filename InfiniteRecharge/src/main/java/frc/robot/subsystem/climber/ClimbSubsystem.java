package frc.robot.subsystem.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class ClimbSubsystem extends BitBucketSubsystem {
    public enum ClimbState {
        Extending, Retracting, Off, Rewinding;
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
        motorLeft.setInverted(true);
        motorLeft.follow(motorRight);
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
                    SmartDashboard.putString(getName() + "/ClimbState", "Retracted");
                    break;

                case Extending:
                    motorRight.set(
                            SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.EXTEND_OUTPUT));
                    SmartDashboard.putString(getName() + "/ClimbState", "Extending");
                    break;

                // no holding state is needed, since a mechanical ratchet will hold the robot
                // while its hanging
                case Retracting:
                    motorRight.set(
                            SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.RETRACT_OUTPUT));
                    SmartDashboard.putString(getName() + "/ClimbState", "Retracting");
                    break;
                case Rewinding:
                    motorRight.set(
                            SmartDashboard.getNumber(getName() + "/Climber Current", ClimbConstants.REWIND_OUTPUT));
                    SmartDashboard.putString(getName() + "/ClimbState", "Rewinding");
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

    public boolean isRewinding(){
        return climbState == ClimbState.Rewinding;
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

    @Override
    public void dashboardInit() {
        super.dashboardInit();
        SmartDashboard.putBoolean(getName() + "/Rewind Enabled", false);
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        SmartDashboard.putNumber(getName() + "/Right Selected Sensor Position", motorRight.getSelectedSensorPosition());
        SmartDashboard.putNumber(getName() + "/Left Selected Sensor Position", motorLeft.getSelectedSensorPosition());
        SmartDashboard.putBoolean(getName() + "/Active", active);
        SmartDashboard.putBoolean(getName() + "/Rewind Enabled", rewindEnabled);
    }

    public void disable() {
    }

    @Override
    public void listTalons() {
        talons.add(motorRight);
        talons.add(motorLeft);
    }
}