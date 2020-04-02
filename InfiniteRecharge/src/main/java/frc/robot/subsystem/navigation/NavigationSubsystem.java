/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.navigation;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.subsystem.navigation.systems.RobotSystem1;
import frc.robot.subsystem.navigation.systems.RobotSystemConfig;
import frc.robot.subsystem.scoring.shooter.ShooterSubsystem;
import frc.robot.utils.data.DoubleDataWindow;
import frc.robot.subsystem.vision.VisionSubsystem;

public class NavigationSubsystem extends BitBucketSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

    private AHRS ahrs;

    // recent acceleration data from IMU
    private DoubleDataWindow imuAcc = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);
    // recent gyroscope data from IMU
    private DoubleDataWindow imuGyro = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);
    // time to update localization stuff
    private DoubleDataWindow dts = new DoubleDataWindow(50);

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;

    // robot system used for localization
    private RobotSystem1 sys;



    // odometry, until we get localization to work, and in case we can't use localization for some reason
    private DifferentialDriveOdometry odometry;


    
	public NavigationSubsystem(Config config, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem) {
        super(config);
        
        this.visionSubsystem = visionSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        
        Rotation2d rotation = new Rotation2d(0);
        odometry = new DifferentialDriveOdometry(
            rotation,
            new Pose2d(0, 0, rotation)
        );
    }
    
    public void setDrive(DriveSubsystem drive) {
        driveSubsystem = drive;
    }

	

	// TODO: provide an accessor that fetches the most current state (6-DOF)
	// and returns it in a structure for reference; this allows consumer
	// to fetch the best available state, at the same time the periodic
	// loop can be keeping a limited history to allow image sensor data
	// to be correlated to past position and angle data; this history
	// would be interpolated if there is sufficient time synchronization
	// between this Robot code and the image data.

	@Override
	public void initialize() {
        super.initialize();

        ahrs = BitBucketsAHRS.instance();

        RobotSystemConfig systemConfig = new RobotSystemConfig();

        systemConfig.azimuthDegrees = shooterSubsystem::getAzimuthDeg;
        systemConfig.leftVolts      = driveSubsystem::getLeftVolts;
        systemConfig.rightVolts     = driveSubsystem::getRightVolts;
        systemConfig.tyFromDistance = visionSubsystem::approximateTyFromDistance;

        sys = new RobotSystem1(config, systemConfig);

        reset();
    }

    public void reset() {
        ahrs.reset();
        ahrs.setAngleAdjustment(0);

        Rotation2d rotation = new Rotation2d(0);//Rotation2d.fromDegrees(getYaw_deg());

        odometry.resetPosition(
            new Pose2d(0, 0, rotation),
            rotation
        );

        driveSubsystem.resetEncoders();
    }

    public void setInitialPose(Pose2d pose) {
        sys.setInitialPose(pose);
    }

  	@Override
	public void testInit() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}

	protected void updateDashboard() {
		//SmartDashboard.putNumber( getName() + "/Yaw Angle (deg)", getYaw_deg());
		//SmartDashboard.putNumber( getName() + "/Yaw Rate (dps)", getYawRate_degPerSec());
	}

	@Override
	public void periodic(float deltaTime) {
		clearDiagnosticsEnabled();		
        updateBaseDashboard();

        


        double acc = Math.sqrt(
            Math.pow(getWorldAccX(), 2) + 
            Math.pow(getWorldAccY(), 2) + 
            Math.pow(getWorldAccZ(), 2)
        );

        double gyro = getGyro();

        


        //imuAcc.add(acc);
        //imuGyro.add(gyro);

        double t0 = System.nanoTime();
        double yaw = getYaw_deg();

        double v0 = 0.01;

        double t1 = System.nanoTime();
        double dt = (t1 - t0) / 1000000000;

        dts.add(dt);
        SmartDashboard.putNumber(getName() + "/processing time (ms)", (t1 - t0) / 1000000);



        odometry.update(
            Rotation2d.fromDegrees(getYaw_deg()),
            driveSubsystem.getLeftDistance_meters(),
            driveSubsystem.getRightDistance_meters()
        );

        if (getTelemetryEnabled()) {
			SmartDashboard.putNumber(getName() + "/Robot yaw", getYaw_deg());
            SmartDashboard.putNumber(getName() + "/Robot raw X accel", getAccX());
            SmartDashboard.putNumber(getName() + "/Robot world X accel", getWorldAccX());
            SmartDashboard.putNumber(getName() + "/Robot raw X gyro", gyro);
            SmartDashboard.putNumber(getName() + "/Robot accel", acc);

            SmartDashboard.putNumber(getName() + "/processing time avg", dts.getAverage());
            SmartDashboard.putNumber(getName() + "/processing time var", dts.getVariance2());

            Translation2d pos = odometry.getPoseMeters().getTranslation();
            SmartDashboard.putNumber(getName() + "/x (m)", pos.getX());
            SmartDashboard.putNumber(getName() + "/y (m)", pos.getY());

            //double accVar = imuAcc.getVariance2();
            //double gyroVar = imuGyro.getVariance2();

            //SmartDashboard.putNumber(getName() + "/Acceleration variance", accVar);
            //SmartDashboard.putNumber(getName() + "/Gyro variance",        gyroVar);

            //SmartDashboard.putNumber(getName() + "/Acceleration avg", imuAcc.getAverage());
            //SmartDashboard.putNumber(getName() + "/Gyro avg",        imuGyro.getAverage());
        }
        
		if (getDiagnosticsEnabled()) {
			
        }

		updateDashboard();
		
	}

	@Override
	public void testPeriodic() {
		// TODO Auto-generated method stub
		
	}

	public double getYaw_deg() {
		return -Math.IEEEremainder(ahrs.getAngle(), 360);
	}
	public double getYawRate_degPerSec() {
		return ahrs.getRate();
	}

	public double getAccX() {
		return ahrs.getRawAccelX();
	}

	public double getWorldAccX() {
		return ahrs.getWorldLinearAccelX();
    }
    
    public double getAccY() {
		return ahrs.getRawAccelY();
	}

	public double getWorldAccY() {
		return ahrs.getWorldLinearAccelY();
    }
    
    public double getAccZ() {
		return ahrs.getRawAccelZ();
	}

	public double getWorldAccZ() {
		return ahrs.getWorldLinearAccelZ();
	}

	public double getGyro() {
		return ahrs.getRawGyroZ();
    }
    


    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public Config getConfig() { return config; }
    public DriveSubsystem getDrive() { return driveSubsystem; }
    public VisionSubsystem getVision() { return visionSubsystem; }
    public ShooterSubsystem getShooter() { return shooterSubsystem; }

    public void disable(){
    }

	@Override
    public void listTalons() {}
}
