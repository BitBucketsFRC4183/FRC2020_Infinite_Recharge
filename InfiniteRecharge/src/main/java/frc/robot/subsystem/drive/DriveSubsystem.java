package frc.robot.subsystem.drive;

import java.util.ArrayList;
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
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.drive.auto.FieldConstants;
import frc.robot.subsystem.drive.auto.FullTrajectory;
import frc.robot.subsystem.drive.auto.commands.CenterThreeBall;
import frc.robot.subsystem.drive.auto.commands.OppTrenchEightBall;
import frc.robot.subsystem.drive.auto.commands.TrenchFiveBall;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.JoystickScale;
import frc.robot.utils.talonutils.MotorUtils;
import frc.robot.subsystem.drive.auto.AutoConfig;

public class DriveSubsystem extends BitBucketSubsystem {

    private DriverStation driverStation;

    // arrays of left and right TalonFXs
    private WPI_TalonFX[] leftMotors;
    private WPI_TalonFX[] rightMotors;

    // navigation so we know information about our location on the field
    private final NavigationSubsystem NAVIGATION_SUBSYSTEM;
    // vision so we can access data from the LL to auto align in commands
    private final VisionSubsystem VISION_SUBSYSTEM;

    // Allow the driver to try different scaling functions on the joysticks
	private static SendableChooser<JoystickScale> forwardJoystickScaleChooser;
    private static SendableChooser<JoystickScale> turnJoystickScaleChooser;
    // for rotation drive, never got used :(
    private static SendableChooser<JoystickScale> rotationJoystickScaleChooser;

    // Let the driver choose the auto path based on where the robot is placed
    private static SendableChooser<FullTrajectory> pickupTrajectoryChooser;
    // List of all the FullTrajectories we will follow
    private final ArrayList<FullTrajectory> trajectories = new ArrayList<FullTrajectory>();

    // store the raw inputs from joysticks for commands to query
    private double rawSpeed = 0, rawTurn = 0;

    // "constants" that may vary per robot
    private final DriveUtils DRIVE_UTILS;

    // RAMSETE controller used during auto for positioning on the field
    private final RamseteController ramsete;

    // PID controllers for the left and right sides of the drive base during auto
    // we tried just using the TalonFX's PID controller that updates at 1kHz, but
    // got bad results that were instantly cured when we used WPI's 50Hz PID.
    // not quite sure what the cause was but it works /shrug
    private final PIDController leftAutoPID;
    private final PIDController rightAutoPID;

    // left and right motors but in a SpeedControllerGroup
    private SpeedControllerGroup leftGroup;
    private SpeedControllerGroup rightGroup;

    // differential drive representing our drive base, used in tracking trajectories
    // during auto
    private DifferentialDrive differentialDrive;

    /**
     * Create an instance of the DriveSubsystem
     *
     * @param config set of configuration values to use
     * @param navigationSubsystem navigation subsystem to get information about our position on the field
     * @param visionSubsystem vision subsystem to get information about the target
     * @param oi Operator Interface so we can easily talk to the joysticks
     */
    public DriveSubsystem(Config config, NavigationSubsystem navigationSubsystem, VisionSubsystem visionSubsystem) {
        super(config);
        NAVIGATION_SUBSYSTEM = navigationSubsystem;
        VISION_SUBSYSTEM = visionSubsystem;

        // create the drive utils
        DRIVE_UTILS = new DriveUtils(config);

        // voltage constraint on the auto path following so we don't ask the motors to do
        // more than they can
        DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
            config.drive.characterization,
            DRIVE_UTILS.KINEMATICS,
            DriveConstants.AUTO_MAX_VOLTAGE
        );

        // kinematics constraint so the code knows how the drive base behaves and the max allowed speed
        DifferentialDriveKinematicsConstraint kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
            DRIVE_UTILS.KINEMATICS,
            config.auto.cruiseSpeed_mps
        );

        // create the configuration for the trajectory using the max speed and acceleration
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            config.auto.cruiseSpeed_mps,
            config.auto.maxAcceleration_mps
        );
        // ... but put in constraints
        trajectoryConfig.addConstraint(kinematicsConstraint);
        trajectoryConfig.addConstraint(voltageConstraint);
        // we start at rest
        trajectoryConfig.setStartVelocity(0);

        // make new trajectory and it to the list
        CenterThreeBall center = new CenterThreeBall(trajectoryConfig);
        trajectories.add(center.getFullTrajectory());

        TrenchFiveBall ourTrench = new TrenchFiveBall(trajectoryConfig);
        trajectories.add(ourTrench.getFullTrajectory());

        OppTrenchEightBall oppTrench = new OppTrenchEightBall(trajectoryConfig);
        trajectories.add(oppTrench.getFullTrajectory());

        ////////////////////////

        // create the RAMSETE controller with the specified tuning parameters
        ramsete = new RamseteController(config.auto.b, config.auto.zeta);

        // create the PID controllers
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

        pickupTrajectoryChooser = new SendableChooser<FullTrajectory>();
        pickupTrajectoryChooser.setDefaultOption(trajectories.get(0).getName(), trajectories.get(0));
        for (int i = 1; i < trajectories.size(); i++) {
            pickupTrajectoryChooser.addOption(trajectories.get(i).getName(), trajectories.get(i));
        }
        SmartDashboard.putData(getName() + "/Pickup Trajectory", pickupTrajectoryChooser);

        // instantiate the left and right motor arrays
        leftMotors = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE];
        rightMotors = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE];

        // and a temporary array with just the follower motors per side
        WPI_TalonFX[] tempLeft = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE - 1];
        WPI_TalonFX[] tempRight = new WPI_TalonFX[config.drive.MOTORS_PER_SIDE - 1];

        // loop through all the motors
        for (int i = 0; i < config.drive.MOTORS_PER_SIDE; i++) {
            // create the motors given their config
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

            // so we can push it to its starting position after testing :)
            leftMotors[i].setNeutralMode(NeutralMode.Coast);
            rightMotors[i].setNeutralMode(NeutralMode.Coast);

            // so we don't head any Falcon500s screaming because they stall too easily
            leftMotors[i].configClosedloopRamp(0.5);
            rightMotors[i].configClosedloopRamp(0.5);



            // so we can compensate for voltage sag from the battery
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

        // they won't accept it if you just give an array like a normal person
        // so you have to take the first index then the rest of the array
        leftGroup = new SpeedControllerGroup(leftMotors[0], tempLeft);
        rightGroup = new SpeedControllerGroup(rightMotors[0], tempRight);

        // create the differential drive with the motors
        differentialDrive = new DifferentialDrive(leftGroup, rightGroup);



        leftMotors[0].setSensorPhase(DriveConstants.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
        rightMotors[0].setSensorPhase(DriveConstants.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);
        leftMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);
        rightMotors[0].selectProfileSlot(MotorUtils.velocitySlot, 0);

    }





    // we used to have closed loop driving but not its open loop
    // this would take a translational and rotational speed and convert them into
    // native velocity units for the Falcons and command it

    // public void velocityDrive_auto(double ips, double radps) {
    //     selectVelocityMode(true);



    //     ips *= config.drive.gearRatio;
    //     double diffSpeed_ips = radps * config.drive.gearRatio * config.drive.trackWidth_in / 2.0;

    //     // Compute, report, and limit lateral acceleration
	// 	if (Math.abs(radps * ips) > DriveConstants.MAX_LAT_ACCELERATION_IPSPS) {
	// 		ips = Math.signum(ips) * DriveConstants.MAX_LAT_ACCELERATION_IPSPS / Math.abs(radps);
	// 	}
	// 	double latAccel_gs = radps * ips / 12.0 / DriveConstants.STANDARD_G_FTPSPS;
    //     double turnRadius_inches = ips / radps;



    //     int speed_tickP100 = DRIVE_UTILS.ipsToTicksP100(ips);
	// 	int diffSpeed_tickP100 = DRIVE_UTILS.ipsToTicksP100(diffSpeed_ips);

	// 	int leftSpeed_tickP100 = speed_tickP100 + diffSpeed_tickP100;
    //     int rightSpeed_tickP100 = speed_tickP100 - diffSpeed_tickP100;

    //     SmartDashboard.putNumber(getName() + "/ls_tp100", leftSpeed_tickP100);
    //     SmartDashboard.putNumber(getName() + "/rs_tp100", rightSpeed_tickP100);

    //     setLeftVelocity(leftSpeed_tickP100);
    //     setRightVelocity(rightSpeed_tickP100);

    //     SmartDashboard.putNumber(getName() + "/commandedSpeed_ips", ips);
    // }

    // drive the robot with a given percent speed and percent turn... worked much better than closed loop
    // so now we're trying to figure out how to bring back closed loop driving because we have dignity
    public void drive(double speed, double turn) {
        speed = forwardJoystickScaleChooser.getSelected().rescale(speed, DriveConstants.JOYSTICK_DEADBAND);
        turn = turnJoystickScaleChooser.getSelected().rescale(turn, DriveConstants.JOYSTICK_DEADBAND);

        leftMotors[0].set(ControlMode.PercentOutput, (speed + turn) * ((config.drive.invertLeftCommand) ? -1 : 1));
        rightMotors[0].set(ControlMode.PercentOutput, (speed - turn) * ((config.drive.invertRightCommand) ? -1 : 1));

        // used to be used to convert to speeds and call velocityDrive_auto()

		// double ips = MathUtils.map(
        //     speed,
        //     -1.0,
        //     1.0,
        //     -config.drive.maxAllowedSpeed_ips,
        //     config.drive.maxAllowedSpeed_ips
        // );

        // double radps = MathUtils.map(
        //     turn,
        //     -1.0,
        //     1.0,
        //     -DRIVE_UTILS.MAX_ROTATION_RADPS,
        //     DRIVE_UTILS.MAX_ROTATION_RADPS
        // );

        // velocityDrive_auto(ips, radps);
    }




    // disable all motors
    public void disable() {
        leftMotors[0].set(ControlMode.PercentOutput, 0.0);
		rightMotors[0].set(ControlMode.PercentOutput, 0.0);
    }




    // calculate the estimated speed (assuming no slippage etc) of the robot given wheel velocities
    public double getSpeed_ips() {
        double leftSpeed = DRIVE_UTILS.ticksP100ToIps(leftMotors[0].getSelectedSensorVelocity());
        double rightSpeed = DRIVE_UTILS.ticksP100ToIps(rightMotors[0].getSelectedSensorVelocity());

        return (leftSpeed + rightSpeed) / 2.0;
    }

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

            SmartDashboard.putNumber(getName() + "/left ticks", leftMotors[0].getSelectedSensorPosition());
            SmartDashboard.putNumber(getName() + "/right ticks", rightMotors[0].getSelectedSensorPosition());

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

	public Trajectory getFirstPickupTrajectory() {
        return pickupTrajectoryChooser.getSelected().getFirstPickupTrajectory();
    }

    public Trajectory getFirstReturnTrajectory() {
        return pickupTrajectoryChooser.getSelected().getFirstReturnTrajectory();
    }

    public boolean hasSecondTrajectory() {
        return pickupTrajectoryChooser.getSelected().hasSecondTrajectory();
    }

    public Trajectory getSecondPickupTrajectory() {
        return pickupTrajectoryChooser.getSelected().getSecondPickupTrajectory();
    }

    public Trajectory getSecondReturnTrajectory() {
        return pickupTrajectoryChooser.getSelected().getSecondReturnTrajectory();
    }

    public Pose2d getPose() {
        return NAVIGATION_SUBSYSTEM.getPose();
    }


    // set the wheel speeds in meters per second so RAMSETE can work
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

    // command a voltage to both motors
    // currently being used to stop both after a trajectory is finished
    // and used by the RAMSETE controller to command a trajectory
    public void tankVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts * ((config.drive.invertLeftCommand) ? -1 : 1));
        rightGroup.setVoltage(rightVolts * ((config.drive.invertRightCommand) ? -1 : 1));
    }

    // left and right volts are used for determining Kalman filter estimates
    public double getLeftVolts() {
        return leftMotors[0].getMotorOutputVoltage();
    }

    public double getRightVolts() {
        return rightMotors[0].getMotorOutputVoltage();
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