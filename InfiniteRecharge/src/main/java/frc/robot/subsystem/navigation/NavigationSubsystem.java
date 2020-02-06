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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.utils.data.DoubleDataWindow;

import frc.robot.subsystem.vision.VisionSubsystem;

/**
 * Add your docs here.
 */
public class NavigationSubsystem extends BitBucketSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

    private AHRS ahrs;

    private DoubleDataWindow imuAcc = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);
    private DoubleDataWindow imuGyro = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);
    private DoubleDataWindow dts = new DoubleDataWindow(50);

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    private RobotSystem sys;



    private DifferentialDriveOdometry odometry;


    
	public NavigationSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        
		this.visionSubsystem = visionSubsystem;
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
        sys = new RobotSystem();

        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(ahrs.getYaw())
        );
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
        double yaw = ahrs.getYaw();

        double v0 = 0.01;

        sys.getModel(v0, yaw);

        double t1 = System.nanoTime();
        double dt = (t1 - t0) / 1000000000;

        dts.add(dt);
        SmartDashboard.putNumber(getName() + "/processing time (ms)", (t1 - t0) / 1000000);



        odometry.update(
            Rotation2d.fromDegrees(ahrs.getYaw()),
            driveSubsystem.getLeftDistance_meters(),
            driveSubsystem.getRightDistance_meters()
        );

        if (getTelemetryEnabled()) {
			SmartDashboard.putNumber(getName() + "/Robot yaw", ahrs.getYaw());
            SmartDashboard.putNumber(getName() + "/Robot raw X accel", getAccX());
            SmartDashboard.putNumber(getName() + "/Robot world X accel", getWorldAccX());
            SmartDashboard.putNumber(getName() + "/Robot raw X gyro", gyro);
            SmartDashboard.putNumber(getName() + "/Robot accel", acc);

            SmartDashboard.putNumber(getName() + "/processing time avg", dts.getAverage());
            SmartDashboard.putNumber(getName() + "/processing time var", dts.getVariance2());

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
		return ahrs.getYaw();
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
}
