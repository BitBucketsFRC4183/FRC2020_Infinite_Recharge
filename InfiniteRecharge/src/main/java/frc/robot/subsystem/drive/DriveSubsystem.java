package frc.robot.subsystem.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.utils.JoystickScale;
import frc.robot.utils.data.filters.RisingEdgeFilter;
import frc.robot.utils.math.MathUtils;
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



    // Allow the driver to try different scaling functions on the joysticks
	private static SendableChooser<JoystickScale> forwardJoystickScaleChooser;
    private static SendableChooser<JoystickScale> turnJoystickScaleChooser;



    public boolean velocityMode;



    private double rawSpeed = 0, rawTurn = 0;
    




    public DriveSubsystem(Config config, NavigationSubsystem navigationSubsystem, OI oi) {
        super(config);
        NAVIGATION_SUBSYSTEM = navigationSubsystem;
        OI = oi;
    }



    public void initialize() {
        initializeBaseDashboard();



        driverStation = DriverStation.getInstance();
        // Make joystick scale chooser and put it on the dashboard
		forwardJoystickScaleChooser = new SendableChooser<JoystickScale>();
		forwardJoystickScaleChooser.setDefaultOption("Linear", JoystickScale.LINEAR);
		forwardJoystickScaleChooser.addOption("Square", JoystickScale.SQUARE);
		forwardJoystickScaleChooser.addOption("Cube", JoystickScale.CUBE);
		forwardJoystickScaleChooser.addOption("Sine", JoystickScale.SINE);

		SmartDashboard.putData(getName() + "/Forward Joystick Scale", forwardJoystickScaleChooser);

		turnJoystickScaleChooser = new SendableChooser<JoystickScale>();
		turnJoystickScaleChooser.addOption("Linear", JoystickScale.LINEAR);
		turnJoystickScaleChooser.setDefaultOption("Square", JoystickScale.SQUARE);
		turnJoystickScaleChooser.addOption("Cube", JoystickScale.CUBE);
		turnJoystickScaleChooser.addOption("Sine", JoystickScale.SINE);
		
		SmartDashboard.putData(getName() + "/Turn Joystick Scale", turnJoystickScaleChooser);



        leftMotors = new WPI_TalonSRX[config.drive.MOTORS_PER_SIDE];
        rightMotors = new WPI_TalonSRX[config.drive.MOTORS_PER_SIDE];

        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            leftMotors[i] = MotorUtils.makeSRX(config.drive.leftMotors[i]);
            rightMotors[i] = MotorUtils.makeSRX(config.drive.rightMotors[i]);

            leftMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
											DriveConstants.HIGH_STATUS_FRAME_PERIOD_MS, 
                                            DriveConstants.CONTROLLER_TIMEOUT_MS);
                                            
            rightMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
											DriveConstants.HIGH_STATUS_FRAME_PERIOD_MS, 
                                            DriveConstants.CONTROLLER_TIMEOUT_MS);
        }

        leftMotors[0].setSensorPhase(DriveConstants.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
        rightMotors[0].setSensorPhase(DriveConstants.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);
        leftMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);
        rightMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);

        setDefaultCommand(new Idle(this));
    }





    public void velocityDrive_auto(double ips, double radps) {
        selectVelocityMode(true);



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
        
        SmartDashboard.putNumber(getName() + "/ls_tp100", leftSpeed_tickP100);
        SmartDashboard.putNumber(getName() + "/rs_tp100", rightSpeed_tickP100);

		leftMotors[0].set(ControlMode.Velocity, leftSpeed_tickP100);
        rightMotors[0].set(ControlMode.Velocity, rightSpeed_tickP100);
        
        SmartDashboard.putNumber(getName() + "/commandedSpeed_ips", ips);
    }

    public void velocityDrive(double speed, double turn) {
        speed = forwardJoystickScaleChooser.getSelected().rescale(speed, DriveConstants.JOYSTICK_DEADBAND);
        turn = turnJoystickScaleChooser.getSelected().rescale(turn, DriveConstants.JOYSTICK_DEADBAND);

        //leftMotors[0].set(ControlMode.PercentOutput, speed + turn);
        //rightMotors[0].set(ControlMode.PercentOutput, speed - turn);
		double ips = MathUtils.map(speed,
            -1.0,
            1.0,
            -DriveConstants.MAX_ALLOWED_SPEED_IPS,
            DriveConstants.MAX_ALLOWED_SPEED_IPS
        );

        double radps = MathUtils.map(turn,
            -1.0,
            1.0,
            -DriveConstants.MAX_ALLOWED_TURN_RADPS,
            DriveConstants.MAX_ALLOWED_TURN_RADPS
        );

        velocityDrive_auto(ips, radps);
    }



    private void selectVelocityMode(boolean needVelocityMode) {
		if (needVelocityMode && !velocityMode) {
			for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
				leftMotors[i].selectProfileSlot(MotorUtils.velocitySlot,
				                                0);
				rightMotors[i].selectProfileSlot(MotorUtils.velocitySlot,
				                                0);
            }
            
			velocityMode = true;
		} else {
			velocityMode = false;
        }
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
                driveMethod = DriveMethod.VELOCITY; //254
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



        if (getTelemetryEnabled()) {
            double leftSpeed = DriveConstants.ticksP100ToIps(leftMotors[0].getSelectedSensorVelocity());
            double rightSpeed = DriveConstants.ticksP100ToIps(rightMotors[0].getSelectedSensorVelocity());
            SmartDashboard.putNumber(getName() + "/leftSpeed_ips", leftSpeed);
            SmartDashboard.putNumber(getName() + "/rightSpeed_ips", rightSpeed);

            SmartDashboard.putNumber(getName() + "/command %", leftMotors[0].getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + "/left vel", leftMotors[0].getSelectedSensorVelocity());
            SmartDashboard.putNumber(getName() + "/speed error", leftMotors[0].getClosedLoopError());
            SmartDashboard.putNumber(getName() + "/left setpoint", leftMotors[0].getClosedLoopTarget());



            SmartDashboard.putNumber(getName() + "/Velocity (in/s)", getApproxV());
            SmartDashboard.putNumber(getName() + "/Omega (rad/s)", getApproxOmega());
        }
    }





    public NavigationSubsystem getNavigation() {
        return NAVIGATION_SUBSYSTEM;
    }





    public void setDriverRawSpeed(double speed) {
        rawSpeed = speed;
    }

    public double getDriverRawSpeed() {
        return rawSpeed;
    }

    public void setDriverRawTurn(double turn) {
        rawTurn = turn;
    }

    public double getDriverRawTurn() {
        return rawTurn;
    }



    public double getApproxV() {
        return 
            (DriveConstants.WHEEL_DIAMETER_INCHES / 2) * 
            (rightMotors[0].getSelectedSensorVelocity() + leftMotors[0].getSelectedSensorVelocity()) / 2.0;
    }

    public double getApproxOmega() {
        return
            (DriveConstants.WHEEL_DIAMETER_INCHES / 2) * 
            (rightMotors[0].getSelectedSensorVelocity() - leftMotors[0].getSelectedSensorVelocity()) / (DriveConstants.WHEEL_TRACK_INCHES / 2.0);
    }
}