package frc.robot.subsystem.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.utils.RisingEdgeFilter;
import frc.robot.utils.talonutils.MotorUtils;



public class DriveSubsystem extends BitBucketSubsystem {
    public enum DriveMethod {
        AUTO,
        IDLE,
        VELOCITY,
        ROTATION
    };
    private DriveMethod driveMethod = DriveMethod.IDLE; // default
    private RisingEdgeFilter driveMethodSwitchFilter = new RisingEdgeFilter();



    private DriverStation driverStation;



    // TODO: CHANGE TO FX WHEN WE GET GOOD BOT
    private WPI_TalonSRX[] leftMotors;
    private WPI_TalonSRX[] rightMotors;

    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;
    private final OI OI;

    public DriveSubsystem(Config config, NavigationSubsystem navigationSubsystem, OI oi) {
        super(config);
        NAVIGATION_SUBSYSTEM = navigationSubsystem;
        OI = oi;
    }



    public void initialize() {
        initializeBaseDashboard();



        driverStation = DriverStation.getInstance();

        leftMotors = new WPI_TalonSRX[config.drive.MOTORS_PER_SIDE];
        rightMotors = new WPI_TalonSRX[config.drive.MOTORS_PER_SIDE];

        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            leftMotors[i] = MotorUtils.makeSRX(config.drive.leftMotors[i]);
            rightMotors[i] = MotorUtils.makeSRX(config.drive.rightMotors[i]);
        }

        leftMotors[0].setSensorPhase(DriveConstants.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
        rightMotors[0].setSensorPhase(DriveConstants.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);
        leftMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);
        rightMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);




        setDefaultCommand(new Idle(this, OI));
    }





    public void velocityDrive(double ips, double radps) {
        double diffSpeed_ips = radps * DriveConstants.WHEEL_TRACK_INCHES / 2.0;

        // Compute, report, and limit lateral acceleration
		if (Math.abs(radps * ips) > DriveConstants.MAX_LAT_ACCELERATION_IPSPS) {
			ips = Math.signum(ips) * DriveConstants.MAX_LAT_ACCELERATION_IPSPS / Math.abs(radps);
		}
		double latAccel_gs = radps * ips / 12.0 / DriveConstants.STANDARD_G_FTPSPS;
        double turnRadius_inches = ips / radps;
        


        int speed_tickP100 = DriveConstants.ipsToTicksP100(ips);
		int diffSpeed_tickP100 = DriveConstants.ipsToTicksP100(diffSpeed_ips);

		int leftSpeed_tickP100 = speed_tickP100 + diffSpeed_tickP100;
		int rightSpeed_tickP100 = speed_tickP100 - diffSpeed_tickP100;

		leftMotors[0].set(ControlMode.Velocity, leftSpeed_tickP100);
		rightMotors[0].set(ControlMode.Velocity, rightSpeed_tickP100);
    }



    public void disable() {
        leftMotors[0].set(ControlMode.PercentOutput, 0.0);
		rightMotors[0].set(ControlMode.PercentOutput, 0.0);
    }





    public double getSpeed_ips() {
        double leftSpeed = DriveConstants.ticksP100ToIps(leftMotors[0].getSelectedSensorVelocity());
        double rightSpeed = DriveConstants.ticksP100ToIps(rightMotors[0].getSelectedSensorVelocity());

        return (leftSpeed + rightSpeed) / 2.0;
    }



    public DriveMethod getDriveMethod() { return driveMethod; }





    public void diagnosticsInitialize() {

    }

    public void diagnosticsPeriodic() {

    }

    public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();



        boolean switchHeld = OI.rotationToVelocity();
        boolean doSwitch = driveMethodSwitchFilter.calculate(switchHeld);

        if (driverStation.isOperatorControl()) {
            if (driveMethod == DriveMethod.AUTO || driveMethod == DriveMethod.IDLE) {
                driveMethod = DriveMethod.VELOCITY;
            }

            if (doSwitch) {
                switch (driveMethod) {
                    case VELOCITY: {
                        driveMethod = DriveMethod.ROTATION;
                        break;
                    }
                    case ROTATION: {
                        driveMethod = DriveMethod.VELOCITY;
                        break;
                    }
                    default: // just keep it I guess? shouldn't get here anyways
                }
            }
        } else if (driverStation.isAutonomous()) {
            driveMethod = DriveMethod.AUTO; // please don't press any buttons during auto anyways :)))
        }



        SmartDashboard.putNumber(getName() + "/Robot yaw", NAVIGATION_SUBSYSTEM.getYaw_deg());
    }





    public NavigationSubsystem getNavigation() {
        return NAVIGATION_SUBSYSTEM;
    }
}