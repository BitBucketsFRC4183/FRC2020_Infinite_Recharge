package frc.robot.utils.dashboard;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.when;
import static org.powermock.api.mockito.PowerMockito.mockStatic;
import static org.powermock.api.mockito.PowerMockito.verifyStatic;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.core.classloader.annotations.SuppressStaticInitializationFor;
import org.powermock.modules.junit4.PowerMockRunner;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.dashboard.annotations.Dashboard;
import frc.robot.utils.dashboard.annotations.Default;
import frc.robot.utils.dashboard.annotations.Key;

@RunWith(PowerMockRunner.class)
@PrepareForTest({ SmartDashboard.class, NetworkTablesJNI.class })
@SuppressStaticInitializationFor({ "edu.wpi.first.wpilibj.smartdashboard.SmartDashboard",
        "edu.wpi.first.networktables.NetworkTablesJNI", "edu.wpi.first.hal.JNIWrapper" })
public class DashboardHandlerTest {

    @Dashboard(value = "Shooter", populateOnCreate = false, getDelaySeconds = 0)
    public interface TestDashboard {
        @Default(5000)
        double shooterTargetRPM();

        @Default(45.5)
        double shooterTargetElevation();

        void shooterPercentOutput(double output);

        @Key("My Custom Key")
        void someOtherValue(boolean value);

        /**
         * This is in the root smartdashboard namespace, not "Shooter/Some Root Value",
         * but just "Some Root Value"
         * 
         * @return
         */
        @Key(value = "Telemetry Enabled", exactPath = true)
        boolean globalTelemetryEnabled();
    }

    /**
     * Create a test Cached dashboard with a read delay of 5 seconds
     * so we can verify the cached dashboard doesn't call SmartDashboard multiple times
     */
    @Dashboard(value = "Shooter", populateOnCreate = false, getDelaySeconds = 1)
    public interface CachedTestDashboard {
        @Default(5000)
        double shooterTargetRPM();
    }

        /**
     * Create a test Cached dashboard with a read delay of 5 seconds
     * so we can verify the cached dashboard doesn't call SmartDashboard multiple times
     */
    @Dashboard(value = "Shooter", populateOnCreate = true, getDelaySeconds = 0)
    public interface PopulateOnCreateTestDashboard {
        @Default(5000)
        double shooterTargetRPM();
    }

    @Before
    public void before() {
        mockStatic(SmartDashboard.class);
    }

    @Test
    public void testKeyForMethodName() throws Exception {
        DashboardHandler handler = new DashboardHandler(CachedTestDashboard.class);

        assertEquals("Shooter/My Method", handler.keyForMethodName("myMethod"));
        assertEquals("Shooter/Output Percent", handler.keyForMethodName("putOutputPercent"));
        assertEquals("Shooter/Output Percent", handler.keyForMethodName("setOutputPercent"));
        assertEquals("Shooter/Output Target", handler.keyForMethodName("getOutputTarget"));
    }

    @Test
    public void testPopulateOnCreate() throws Exception {
        DashboardFactory.create(PopulateOnCreateTestDashboard.class);

        // Make sure our default values were populated on creation of the Dashboard object
        verifyStatic(SmartDashboard.class);
        SmartDashboard.putNumber(eq("Shooter/Shooter Target RPM"), eq(5000.0));
    }

    @Test
    public void testDashboardGet() throws Exception {
        TestDashboard testDashboard = DashboardFactory.create(TestDashboard.class);

        // This should call "getNumber" for the shooterTargetRPM
        testDashboard.shooterTargetRPM();
        verifyStatic(SmartDashboard.class);
        SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), eq(5000.0));

        // Make the SmartDashboard return 4000 and verify our dashboard is updated
        when(SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble())).thenReturn(4000.0);
        double rpm = testDashboard.shooterTargetRPM();
        assertEquals(4000, rpm, 0);

        // verify our default works for elevation angle
        when(SmartDashboard.getNumber(eq("Shooter/Shooter Target Elevation"), eq(45.5))).thenReturn(45.5);
        double elevationAngle = testDashboard.shooterTargetElevation();
        assertEquals(45.5, elevationAngle, 0);
    }

    @Test
    public void testDashboardPut() throws Exception {
        TestDashboard testDashboard = DashboardFactory.create(TestDashboard.class);

        // make sure the setter works
        testDashboard.shooterPercentOutput(10);
        verifyStatic(SmartDashboard.class);
        SmartDashboard.putNumber(eq("Shooter/Shooter Percent Output"), eq(10.0));
    }

    @Test
    public void testNonDefault() throws Exception {
        TestDashboard testDashboard = DashboardFactory.create(TestDashboard.class);

        when(SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble())).thenReturn(1000.0);
        double rpm = testDashboard.shooterTargetRPM();
        assertEquals(1000, rpm, 0);

    }

    @Test
    public void testCache() throws Exception {
        CachedTestDashboard testDashboard = DashboardFactory.create(CachedTestDashboard.class);

        // first time should return 1000, verify it was called once
        when(SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble())).thenReturn(1000.0);
        assertEquals(1000, testDashboard.shooterTargetRPM(), 0);
        verifyStatic(SmartDashboard.class, times(1));
        SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble());

        // Call it again immediately and verify the smart dashboard was not called again
        mockStatic(SmartDashboard.class);
        testDashboard.shooterTargetRPM();
        testDashboard.shooterTargetRPM();
        verifyStatic(SmartDashboard.class, never());
        SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble());

        // one last call to verify it gets called again after a delay
        Thread.sleep(1000);
        testDashboard.shooterTargetRPM();
        verifyStatic(SmartDashboard.class, times(1));
        SmartDashboard.getNumber(eq("Shooter/Shooter Target RPM"), anyDouble());

    }

    @Test
    public void testKey() throws Exception {
        TestDashboard testDashboard = DashboardFactory.create(TestDashboard.class);
        // clear out the static because our dashboard initializes all get vars
        // on instantiation
        mockStatic(SmartDashboard.class);

        // write the boolean
        testDashboard.someOtherValue(true);
        verifyStatic(SmartDashboard.class);
        SmartDashboard.putBoolean(eq("Shooter/My Custom Key"), eq(true));

        // make the SmartDashboard return true
        when(SmartDashboard.getBoolean(eq("Telemetry Enabled"), anyBoolean())).thenReturn(true);
        assertEquals(true, testDashboard.globalTelemetryEnabled());
    }
}
