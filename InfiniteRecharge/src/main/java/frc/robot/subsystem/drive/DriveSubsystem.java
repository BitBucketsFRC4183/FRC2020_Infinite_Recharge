package frc.robot.subsystem.drive;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.drive.auto.FieldConstants;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.JoystickScale;
import frc.robot.utils.data.filters.RisingEdgeFilter;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.talonutils.MotorUtils;

import frc.robot.subsystem.drive.RamseteSpecial;



public class DriveSubsystem extends BitBucketSubsystem {
    public enum DriveMethod {
        AUTO,
        IDLE,
        VELOCITY,
        ROTATION,
        ALIGN
    };
    private DriveMethod driveMethod = DriveMethod.IDLE; // default
    private RisingEdgeFilter driveMethodSwitchFilter = new RisingEdgeFilter();



    private DriverStation driverStation;



    private WPI_TalonFX[] leftMotors;
    private WPI_TalonFX[] rightMotors;

    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;
    private final VisionSubsystem VISION_SUBSYSTEM;
    private final OI OI;



    // Allow the driver to try different scaling functions on the joysticks
	private static SendableChooser<JoystickScale> forwardJoystickScaleChooser;
    private static SendableChooser<JoystickScale> turnJoystickScaleChooser;
    private static SendableChooser<JoystickScale> rotationJoystickScaleChooser;



    public boolean velocityMode;



    private double rawSpeed = 0, rawTurn = 0;



    private final DriveUtils DRIVE_UTILS;




    private final Trajectory autoTrajectory;
    private final RamseteController ramsete;

    private final PIDController leftAutoPID;
    private final PIDController rightAutoPID;

    private SpeedControllerGroup leftGroup;
    private SpeedControllerGroup rightGroup;

    private DifferentialDrive differentialDrive;
    




    public DriveSubsystem(Config config, NavigationSubsystem navigationSubsystem, VisionSubsystem visionSubsystem, OI oi) {
        super(config);
        NAVIGATION_SUBSYSTEM = navigationSubsystem;
        VISION_SUBSYSTEM = visionSubsystem;
        OI = oi;

        DRIVE_UTILS = new DriveUtils(config);

        DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
            config.drive.characterization,
            DRIVE_UTILS.KINEMATICS,
            DriveConstants.AUTO_MAX_VOLTAGE
        );

        DifferentialDriveKinematicsConstraint kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
            DRIVE_UTILS.KINEMATICS,
            config.auto.cruiseSpeed_mps
        );

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            config.auto.cruiseSpeed_mps,
            config.auto.maxAcceleration_mps
        );
        trajectoryConfig.addConstraint(kinematicsConstraint);
        trajectoryConfig.addConstraint(voltageConstraint);

        autoTrajectory = TrajectoryGenerator.generateTrajectory(
             // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 0.5),
                new Translation2d(4, 0.75)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(5, 1, new Rotation2d(0)),
            trajectoryConfig
        );

        ramsete = new RamseteController(config.auto.b, config.auto.zeta);

        leftAutoPID = new PIDController(config.auto.kP, 0, 0);
        rightAutoPID = new PIDController(config.auto.kP, 0, 0);
    }



    public void initialize() {
        dashboardInit();



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
        
        rotationJoystickScaleChooser = new SendableChooser<JoystickScale>();
        rotationJoystickScaleChooser.addOption("Linear", JoystickScale.LINEAR);
		rotationJoystickScaleChooser.setDefaultOption("Square", JoystickScale.SQUARE);
		rotationJoystickScaleChooser.addOption("Cube", JoystickScale.CUBE);
		rotationJoystickScaleChooser.addOption("Sine", JoystickScale.SINE);
		
        SmartDashboard.putData(getName() + "/Rotation Joystick Scale", turnJoystickScaleChooser);



        leftMotors = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE];
        rightMotors = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE];

        WPI_TalonFX[] tempLeft = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE - 1];
        WPI_TalonFX[] tempRight = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE - 1];

        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            leftMotors[i] = MotorUtils.makeFX(config.drive.leftMotors[i]);
            rightMotors[i] = MotorUtils.makeFX(config.drive.rightMotors[i]);

            leftMotors[i].setStatusFramePeriod(
                StatusFrameEnhanced.Status_13_Base_PIDF0, 
				DriveConstants.HIGH_STATUS_FRAME_PERIOD_MS, 
                DriveConstants.CONTROLLER_TIMEOUT_MS
            );
                                            
            rightMotors[i].setStatusFramePeriod(
                StatusFrameEnhanced.Status_13_Base_PIDF0, 
				DriveConstants.HIGH_STATUS_FRAME_PERIOD_MS, 
                DriveConstants.CONTROLLER_TIMEOUT_MS
            );

            leftMotors[i].setNeutralMode(NeutralMode.Coast);
            rightMotors[i].setNeutralMode(NeutralMode.Coast);

            leftMotors[i].configClosedloopRamp(0.5);
            rightMotors[i].configClosedloopRamp(0.5);



            leftMotors[i].enableVoltageCompensation(true);
            leftMotors[i].configVoltageCompSaturation(DriveConstants.MAX_VOLTS);

            rightMotors[i].enableVoltageCompensation(true);
            rightMotors[i].configVoltageCompSaturation(DriveConstants.MAX_VOLTS);

            // I despise WPI
            if (i != 0) {
                tempLeft[i - 1] = leftMotors[i];
                tempRight[i - 1] = rightMotors[i];
            }
        }

        leftGroup = new SpeedControllerGroup(leftMotors[0], tempLeft);
        rightGroup = new SpeedControllerGroup(rightMotors[0], tempRight);

        differentialDrive = new DifferentialDrive(leftGroup, rightGroup);



        leftMotors[0].setSensorPhase(DriveConstants.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
        rightMotors[0].setSensorPhase(DriveConstants.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);
        leftMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);
        rightMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);

        setDefaultCommand(new Idle(this));
    }





    public void velocityDrive_auto(double ips, double radps) {
        selectVelocityMode(true);



        ips *= config.drive.gearRatio;
        double diffSpeed_ips = radps * config.drive.gearRatio * config.drive.trackWidth_in / 2.0;

        // Compute, report, and limit lateral acceleration
		if (Math.abs(radps * ips) > DriveConstants.MAX_LAT_ACCELERATION_IPSPS) {
			ips = Math.signum(ips) * DriveConstants.MAX_LAT_ACCELERATION_IPSPS / Math.abs(radps);
		}
		double latAccel_gs = radps * ips / 12.0 / DriveConstants.STANDARD_G_FTPSPS;
        double turnRadius_inches = ips / radps;
        


        int speed_tickP100 = DRIVE_UTILS.ipsToTicksP100(ips);
		int diffSpeed_tickP100 = DRIVE_UTILS.ipsToTicksP100(diffSpeed_ips);

		int leftSpeed_tickP100 = speed_tickP100 + diffSpeed_tickP100;
        int rightSpeed_tickP100 = speed_tickP100 - diffSpeed_tickP100;
        
        SmartDashboard.putNumber(getName() + "/ls_tp100", leftSpeed_tickP100);
        SmartDashboard.putNumber(getName() + "/rs_tp100", rightSpeed_tickP100);

        setLeftVelocity(leftSpeed_tickP100);
        setRightVelocity(rightSpeed_tickP100);
        
        SmartDashboard.putNumber(getName() + "/commandedSpeed_ips", ips);
    }

    public void velocityDrive(double speed, double turn) {
        speed = forwardJoystickScaleChooser.getSelected().rescale(speed, DriveConstants.JOYSTICK_DEADBAND);
        turn = turnJoystickScaleChooser.getSelected().rescale(turn, DriveConstants.JOYSTICK_DEADBAND);

		double ips = MathUtils.map(
            speed,
            -1.0,
            1.0,
            -config.drive.maxAllowedSpeed_ips,
            config.drive.maxAllowedSpeed_ips
        );

        double radps = MathUtils.map(
            turn,
            -1.0,
            1.0,
            -DRIVE_UTILS.MAX_ROTATION_RADPS,
            DRIVE_UTILS.MAX_ROTATION_RADPS
        );

        velocityDrive_auto(ips, radps);
    }

    public void rotationDrive(double speed, double turn, double yaw0) {
        speed = forwardJoystickScaleChooser.getSelected().rescale(speed, DriveConstants.JOYSTICK_DEADBAND);
        turn = rotationJoystickScaleChooser.getSelected().rescale(turn, DriveConstants.JOYSTICK_DEADBAND);

        double ips = MathUtils.map(
            speed,
            -1.0,
            1.0,
            -config.drive.maxAllowedSpeed_ips,
            config.drive.maxAllowedSpeed_ips
        );

        double offset = MathUtils.map(
            turn,
            -1.0,
            1.0,
            -DriveConstants.ROTATION_DRIVE_MAX_OFFSET_DEG,
            DriveConstants.ROTATION_DRIVE_MAX_OFFSET_DEG
        );


        double yaw = NAVIGATION_SUBSYSTEM.getYaw_deg();
        SmartDashboard.putNumber(getName() + "/yaw", yaw);
        double yawCommand = yaw0 + offset;
        SmartDashboard.putNumber(getName() + "/yaw command", yawCommand);

        double yawError = yaw - yawCommand;
        SmartDashboard.putNumber(getName() + "/yaw error", yawError);

        yawError = (yawError + 720.0) % (360.0);
        if (yawError > 180) {
            // additional = yawError - 180
            // -180 + additional
            yawError -= 360;
        }

        double omega = yawError * config.drive.ROTATION_DRIVE_KP;



        velocityDrive_auto(ips, omega);
    }



    private void selectVelocityMode(boolean needVelocityMode) {
		if (needVelocityMode && !velocityMode) {
			for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
				leftMotors[i].selectProfileSlot(MotorUtils.velocitySlot, 0);
				rightMotors[i].selectProfileSlot(MotorUtils.velocitySlot, 0);
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
        double leftSpeed = DRIVE_UTILS.ticksP100ToIps(leftMotors[0].getSelectedSensorVelocity());
        double rightSpeed = DRIVE_UTILS.ticksP100ToIps(rightMotors[0].getSelectedSensorVelocity());

        return (leftSpeed + rightSpeed) / 2.0;
    }



    public DriveMethod getDriveMethod() { return driveMethod; }





    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();



        differentialDrive.feed();



        boolean switchHeld = OI.rotationToVelocity();
        boolean doSwitch = driveMethodSwitchFilter.calculate(switchHeld);

        if (driverStation.isOperatorControl()) {
            if (driveMethod == DriveMethod.AUTO || driveMethod == DriveMethod.IDLE) {
                driveMethod = DriveMethod.VELOCITY;
            }

            // won't happen by AZ North
            // if (doSwitch) {
            //     switch (driveMethod) {
            //         case VELOCITY: {
            //             //driveMethod = DriveMethod.ROTATION;
            //             break;
            //         }
            //         case ROTATION: {
            //             driveMethod = DriveMethod.VELOCITY;
            //             break;
            //         }
            //         default: // just keep it I guess? shouldn't get here anyways
            //     }
            // }

            if (autoAligning) {
                driveMethod = DriveMethod.ALIGN;
            } else {
                driveMethod = DriveMethod.VELOCITY;
            }
        } else if (driverStation.isAutonomous()) {
            driveMethod = DriveMethod.AUTO; // please don't press any buttons during auto anyways :)))
        }



        if (getTelemetryEnabled()) {
            double leftSpeed = DRIVE_UTILS.ticksP100ToIps(leftMotors[0].getSelectedSensorVelocity());
            double rightSpeed = DRIVE_UTILS.ticksP100ToIps(rightMotors[0].getSelectedSensorVelocity());
            SmartDashboard.putNumber(getName() + "/leftSpeed_ips", leftSpeed);
            SmartDashboard.putNumber(getName() + "/rightSpeed_ips", rightSpeed);

            SmartDashboard.putNumber(getName() + "/left command %", leftMotors[0].getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + "/left vel", leftMotors[0].getSelectedSensorVelocity());
            SmartDashboard.putNumber(getName() + "/left speed error", leftMotors[0].getClosedLoopError());
            SmartDashboard.putNumber(getName() + "/left setpoint", leftMotors[0].getClosedLoopTarget());

            SmartDashboard.putNumber(getName() + "/right command %", rightMotors[0].getMotorOutputPercent());
            SmartDashboard.putNumber(getName() + "/right  vel", rightMotors[0].getSelectedSensorVelocity());
            SmartDashboard.putNumber(getName() + "/right speed error", rightMotors[0].getClosedLoopError());
            SmartDashboard.putNumber(getName() + "/right  setpoint", rightMotors[0].getClosedLoopTarget());



            SmartDashboard.putNumber(getName() + "/Velocity (in/s)", getApproxV());
            SmartDashboard.putNumber(getName() + "/Omega (rad/s)", getApproxOmega());

            SmartDashboard.putNumber(getName() + "/left ticks", leftMotors[0].getSelectedSensorPosition());
            SmartDashboard.putNumber(getName() + "/right ticks", rightMotors[0].getSelectedSensorPosition());

            SmartDashboard.putString(getName() + "/drive method", driveMethod.toString());
        }
    }





    public NavigationSubsystem getNavigation() {
        return NAVIGATION_SUBSYSTEM;
    }

    public VisionSubsystem getVision() {
        return VISION_SUBSYSTEM;
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

    private boolean autoAligning = false;
    public void setAutoAligning(boolean aligning) {
        autoAligning = aligning;
    }

    public boolean getAutoAligning() { return autoAligning;}



    public int getLeftVelocity_tp100ms() {
        return ((config.drive.invertLeftCommand) ? -1 : 1) * leftMotors[0].getSelectedSensorVelocity();
    }

    public int getRightVelocity_tp100ms() {
        return ((config.drive.invertRightCommand) ? -1 : 1) * rightMotors[0].getSelectedSensorVelocity();
    }

    public double getApproxV() {
        return 
            config.drive.wheelRadius_in * 
            (getLeftVelocity_tp100ms() + getRightVelocity_tp100ms()) / 2.0;
    }

    public double getApproxOmega() {
        return
            config.drive.wheelRadius_in * 
            (getRightVelocity_tp100ms() - getLeftVelocity_tp100ms()) / (config.drive.trackWidth_in / 2.0);
    }



    public double getLeftDistance_meters() {
        return ((config.drive.invertLeftCommand) ? -1 : 1) * leftMotors[0].getSelectedSensorPosition() * DRIVE_UTILS.WHEEL_CIRCUMFERENCE_INCHES / (config.drive.gearRatio * config.drive.ticksPerRevolution) * DriveConstants.METERS_PER_INCH;
    }

    public double getRightDistance_meters() {
        return ((config.drive.invertRightCommand) ? -1 : 1) * rightMotors[0].getSelectedSensorPosition() * DRIVE_UTILS.WHEEL_CIRCUMFERENCE_INCHES / (config.drive.gearRatio * config.drive.ticksPerRevolution) * DriveConstants.METERS_PER_INCH;
    }

    public double getLeftVelocity_mps() {
        return DRIVE_UTILS.ticksP100ToIps(getLeftVelocity_tp100ms()) * DriveConstants.METERS_PER_INCH / config.drive.gearRatio;
    }

    public double getRightVelocity_mps() {
        return DRIVE_UTILS.ticksP100ToIps(getRightVelocity_tp100ms()) * DriveConstants.METERS_PER_INCH / config.drive.gearRatio;
    }

	public Trajectory getAutoTrajectory() {
		return autoTrajectory;
    }
    
    public Pose2d getPose() {
        return NAVIGATION_SUBSYSTEM.getPose();
    }



	public void setWheelSpeeds(double leftSpeed_mps, double rightSpeed_mps) {
        double leftTps = DRIVE_UTILS.ipsToTicksP100(leftSpeed_mps / DriveConstants.METERS_PER_INCH);
        double rightTps = DRIVE_UTILS.ipsToTicksP100(rightSpeed_mps / DriveConstants.METERS_PER_INCH);

        setLeftVelocity(leftTps);
        setRightVelocity(rightTps);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity_mps(), getRightVelocity_mps());
    }
    
    public DifferentialDriveKinematics getKinematics() {
        return DRIVE_UTILS.KINEMATICS;
    }



    private void setLeftVelocity(double vel_tp100ms) {
        leftMotors[0].set(ControlMode.Velocity, ((config.drive.invertLeftCommand) ? -1 : 1) * vel_tp100ms);
    }

    private void setRightVelocity(double vel_tp100ms) {
        rightMotors[0].set(ControlMode.Velocity, ((config.drive.invertRightCommand) ? -1 : 1) * vel_tp100ms);
    }

    public RamseteController getRAMSETEController() {
        return ramsete;
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public SimpleMotorFeedforward getCharacterization() {
        return config.drive.characterization;
    }

    public PIDController getLeftAutoPID() { return leftAutoPID; }
    public PIDController getRightAutoPID() { return rightAutoPID; }

    public void tankVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts * ((config.drive.invertLeftCommand) ? -1 : 1));
        rightGroup.setVoltage(rightVolts * ((config.drive.invertRightCommand) ? -1 : 1));
    }



    public void resetEncoders() {
        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            leftMotors[i].setSelectedSensorPosition(0);
            rightMotors[i].setSelectedSensorPosition(0);
        }
    }



    @Override
	public void listTalons() {
        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            talons.add(leftMotors[i]);
            talons.add(rightMotors[i]);
        }
    }
}