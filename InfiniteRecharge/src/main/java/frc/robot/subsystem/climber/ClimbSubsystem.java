package frc.robot.subsystem.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.dashboard.DashboardFactory;
import frc.robot.utils.talonutils.MotorUtils;

public class ClimbSubsystem extends BitBucketSubsystem {

    private final ClimbDashboard climbDashboard;
    private boolean active = false;
    protected WPI_TalonSRX motorRight;
    protected WPI_TalonSRX motorLeft;

    public ClimbSubsystem(Config config) {
        super(config);
        climbDashboard = DashboardFactory.create(ClimbDashboard.class);
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
    }

    public void pitRewind(double leftStick, double rightStick) {
        if (!isPitRewindEnabled()) {
            return;
        }

        // invert because otherwise down is positive
        leftStick *= -1;
        rightStick *= -1;

        motorLeft.set(climbDashboard.rewindOutput() * leftStick);
        motorRight.set(climbDashboard.rewindOutput() * rightStick);
    }

    public void toggleActive() {
        active = !active;
    }

    public void disableClimb() {
        active = false;
    }

    public void disableRewind() {
        climbDashboard.putRewindEnabled(false);
    }

    // true if pit rewind
    // false if manual rewind
    public boolean isPitRewindEnabled(){
        return climbDashboard.rewindEnabled();
    }

    public boolean isActive(){
        return active;
    }

    public void moveArms(double leftStick, double rightStick) {
        if (!isActive()) {
            return;
        }

        if (isPitRewindEnabled()) {
            pitRewind(leftStick, rightStick);
        } else {
            manualClimb(leftStick, rightStick);
        }
    }

    public void manualClimb(double leftStick, double rightStick){
        leftStick = Math.abs(leftStick);
        rightStick = Math.abs(rightStick);

        if (leftStick > ClimbConstants.DEADBAND) {
            motorLeft.set(ControlMode.PercentOutput, leftStick * climbDashboard.windOutput());
        } else {
            motorLeft.set(ControlMode.PercentOutput, 0);
        }
        if (rightStick > ClimbConstants.DEADBAND) {
            motorRight.set(ControlMode.PercentOutput, rightStick * climbDashboard.windOutput());
        } else {
            motorRight.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void dashboardInit() {
        super.dashboardInit();
        disableRewind();

    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        climbDashboard.putLeftMotorOutput(motorLeft.getMotorOutputPercent());
        climbDashboard.putRightMotorOutput(motorRight.getMotorOutputPercent());
        climbDashboard.putActive(active);
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