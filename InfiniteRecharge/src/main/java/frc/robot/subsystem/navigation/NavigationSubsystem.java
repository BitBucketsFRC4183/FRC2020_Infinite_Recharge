/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.navigation;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.data.DoubleDataWindow;

/**
 * Add your docs here.
 */
public class NavigationSubsystem extends BitBucketSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.



    DoubleDataWindow imuAcc = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);
    DoubleDataWindow imuGyro = new DoubleDataWindow(NavigationConstants.IMU_DATA_SIZE);


    
	public NavigationSubsystem(Config config) {
		super(config);
	}

	private AHRS ahrs;

	// TODO: provide an accessor that fetches the most current state (6-DOF)
	// and returns it in a structure for reference; this allows consumer
	// to fetch the best available state, at the same time the periodic
	// loop can be keeping a limited history to allow image sensor data
	// to be correlated to past position and angle data; this history
	// would be interpolated if there is sufficient time synchronization
	// between this Robot code and the image data.

	@Override
	public void initialize() {
		initializeBaseDashboard();
		ahrs = BitBucketsAHRS.instance();
	}

  	@Override
	public void diagnosticsInitialize() {
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


        imuAcc.add(acc);
        imuGyro.add(gyro);



		if (getTelemetryEnabled()) {
			SmartDashboard.putNumber(getName() + "/Robot yaw", getYaw_deg());
            SmartDashboard.putNumber(getName() + "/Robot raw X accel", getAccX());
            SmartDashboard.putNumber(getName() + "/Robot world X accel", getWorldAccX());
            SmartDashboard.putNumber(getName() + "/Robot raw X gyro", gyro);
            SmartDashboard.putNumber(getName() + "/Robot accel", acc);

            double accVar = imuAcc.getVariance();
            double gyroVar = imuGyro.getVariance();

            SmartDashboard.putNumber(getName() + "/Aceleration variance", accVar);
            SmartDashboard.putNumber(getName() + "/Gyro variance",        gyroVar);
        }
        
		if (getDiagnosticsEnabled()) {
			
        }

		updateDashboard();
		
	}

	@Override
	public void diagnosticsPeriodic() {
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
}
