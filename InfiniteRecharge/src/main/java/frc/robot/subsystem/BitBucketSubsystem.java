/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.subsystem.SubsystemUtilities.DiagnosticsState;

/**
 *
 */
public abstract class BitBucketSubsystem extends SubsystemBase {

	protected final DriverStation driverStation; // Convenience

	protected final Config config;

	protected boolean initializedBase = false;

	// We require that extended telementry and diagnostics enabling
	// reuturn to "OFF" at each reset. Because SendableChoosers remember
	// their state in the NetworkTable (somewhere) we need to use
	// boolean types for simple, resettable switches.
	protected boolean telemetryEnabled = false; // Really an "extended" telemetry set (more bandwidth)
	protected boolean diagnosticsEnabled = false;

	public DiagnosticsState lastKnownState = DiagnosticsState.UNKNOWN;
	public int DIAG_LOOPS_RUN = 5;

	// The time period between calls to Periodic() functions.
	public static double period;

	protected int periodicCounter = 0;

	public BitBucketSubsystem(Config config) {
		setName(getClass().getSimpleName());
		this.config = config;
		this.driverStation = DriverStation.getInstance();
	}

	protected void dashboardInit() {
		SmartDashboard.putBoolean(getName() + "/TelemetryEnabled", telemetryEnabled);
		SmartDashboard.putBoolean(getName() + "/DiagnosticsEnabled", diagnosticsEnabled);

		initializedBase = true;
		SmartDashboard.putBoolean(getName() + "/InitializedBase", initializedBase);
	}

	/** updateBaseDashboard - call from derived class periodic function */
	protected void updateBaseDashboard() {
		SmartDashboard.putNumber(getName() + "/PeriodicCounter", periodicCounter++);
		if (getCurrentCommand() != null) {
			SmartDashboard.putString(getName() + "/CurrentCommand", getCurrentCommand().getName());
		}
	}

	/**
	 * getTelementryEnabled - returns the current dashboard state
	 * 
	 * NOTE: "Extended" Telemetry can be enabled any time at the expense of network
	 * bandwidth
	 */
	public boolean getTelemetryEnabled() {
		telemetryEnabled = SmartDashboard.getBoolean(getName() + "/TelemetryEnabled", false);
		return telemetryEnabled;
	}

	/**
	 * getDiagnosticsEnabled - returns the current dashboard state
	 * 
	 * NOTE: Diagnostics can only be enabled when the DriverStation is in test mode
	 */
	public boolean getDiagnosticsEnabled() {
		diagnosticsEnabled = SmartDashboard.getBoolean(getName() + "/DiagnosticsEnabled", false);
		if (!driverStation.isTest()) {
			diagnosticsEnabled = false;
			SmartDashboard.putBoolean(getName() + "/DiagnosticsEnabled", diagnosticsEnabled);
		}
		return diagnosticsEnabled;
	}

	public void clearDiagnosticsEnabled() {
		diagnosticsEnabled = false;
		SmartDashboard.putBoolean(getName() + "/DiagnosticsEnabled", diagnosticsEnabled);
	}

	public void initialize() {
		dashboardInit();
		testInit();
	};

	// Force all derived classes to have these interfaces
	public abstract void testInit();

	public abstract void testPeriodic();

	public abstract void diagnosticsCheck();

    public abstract void periodic(float deltaTime);

    public abstract void dashboardPeriodic(float deltaTime);

    public abstract void disable();

    // list of Talons this subsystem uses, used in CANChecker
    protected List<BaseTalon> talons = new ArrayList<BaseTalon>();
    /**
	 * Child subsystems override this to add their BaseTalon to the "talons" list
	 * for use by the CANChecker
	 */
    public abstract void listTalons();
    // return the talons being used so the CANChecker has a list of Talons to deal with
    public List<BaseTalon> getTalons() { return talons; }
}