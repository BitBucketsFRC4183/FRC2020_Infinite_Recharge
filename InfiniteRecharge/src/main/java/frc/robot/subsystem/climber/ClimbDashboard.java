package frc.robot.subsystem.climber;

import frc.robot.utils.dashboard.annotations.Dashboard;
import frc.robot.utils.dashboard.annotations.Default;
import frc.robot.utils.dashboard.annotations.DefaultBoolean;

/**
 * This interface is used to interace with the SmartDashboard
 * It automatically caches "get" calls to only refresh every 1 second, so we can call
 * them every 20ms loop but it'll only actually call the SmartDashboard every 50 loops (approximately)
 */
@Dashboard(getDelaySeconds = 1)
public interface ClimbDashboard {
    /**
     * @return true if rewind is enabled, defaults to false, always
     */
    @DefaultBoolean(false)
    boolean rewindEnabled();

    /**
     * @return Get the percent to run the climb motors at
     */
    @Default(ClimbConstants.OUTPUT)
    double windOutput();

    /**
     * @return Get the percent to run the climb motors at
     */
    @Default(ClimbConstants.REWIND_OUTPUT)
    double rewindOutput();

    /**
     * Enable or disable rewindEnabled
     * We only have a dashboard entry for this because we disable it during teleopInit
     * @param enabled Whether to disable or enable the rewind mode
     */
    void putRewindEnabled(boolean enabled);

    // These are all functions to output telemetry to the smart dashboard
    void putActive(boolean active);
    void putRightMotorOutput(double output);
    void putLeftMotorOutput(double output);
}
