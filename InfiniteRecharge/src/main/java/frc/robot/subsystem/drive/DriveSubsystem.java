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
import frc.robot.utils.talonutils.TalonUtils;



public class DriveSubsystem extends BitBucketSubsystem {
    public enum DriveMethod {
        AUTO,
        IDLE,
        VELOCITY,
        ROTATION
    };
    private DriveMethod driveMethod = DriveMethod.IDLE; // default
    private RisingEdgeFilter driveMethodSwitchFilter = new RisingEdgeFilter();



    private DriverStation ds;



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



        ds = DriverStation.getInstance();



        // left and right leader motor device IDs
        int leftLeader = DriveConstants.LEFT_MOTOR_IDS[0];
        int rightLeader = DriveConstants.RIGHT_MOTOR_IDS[0];

        leftMotors = new WPI_TalonSRX[DriveConstants.MOTORS_PER_SIDE];
        rightMotors = new WPI_TalonSRX[DriveConstants.MOTORS_PER_SIDE];
        
        // initialize all motors
        for (int i = 0; i < DriveConstants.MOTORS_PER_SIDE; i++) {
            leftMotors[i] = new WPI_TalonSRX(DriveConstants.LEFT_MOTOR_IDS[i]);
            rightMotors[i] = new WPI_TalonSRX(DriveConstants.RIGHT_MOTOR_IDS[i]);

            // reset to factory defaults
            TalonUtils.initializeMotorDefaults(leftMotors[i]);
            TalonUtils.initializeMotorDefaults(rightMotors[i]);

            // set follower to corresponding leader if not already the leader
            if (i != 0) {
                leftMotors[i].set(ControlMode.Follower, leftLeader);
                rightMotors[i].set(ControlMode.Follower, rightLeader);
            }
        }



        // initialize PID for leaders

        TalonUtils.initializeMotorFPID(
            leftMotors[0],
            DriveConstants.LEFT_VEL_KF,
            DriveConstants.LEFT_VEL_KP,
            DriveConstants.LEFT_VEL_KI,
            DriveConstants.LEFT_VEL_KD,
            DriveConstants.LEFT_VEL_IZONE,
            DriveConstants.VELOCITY_IDX
        );

        TalonUtils.initializeMotorFPID(
            rightMotors[0],
            DriveConstants.RIGHT_VEL_KF,
            DriveConstants.RIGHT_VEL_KP,
            DriveConstants.RIGHT_VEL_KI,
            DriveConstants.RIGHT_VEL_KD,
            DriveConstants.RIGHT_VEL_IZONE,
            DriveConstants.VELOCITY_IDX
        );

        leftMotors[0].setSensorPhase(DriveConstants.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
        rightMotors[0].setSensorPhase(DriveConstants.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);



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
    public void periodic() {
        updateBaseDashboard();



        boolean switchHeld = OI.rotationToVelocity();
        boolean doSwitch = driveMethodSwitchFilter.calculate(switchHeld);

        if (ds.isOperatorControl()) {
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
        } else if (ds.isAutonomous()) {
            driveMethod = DriveMethod.AUTO; // please don't press any buttons during auto anyways :)))
        }



        SmartDashboard.putNumber(getName() + "/Robot yaw", NAVIGATION_SUBSYSTEM.getYaw_deg());
    }





    public NavigationSubsystem getNavigation() {
        return NAVIGATION_SUBSYSTEM;
    }
}