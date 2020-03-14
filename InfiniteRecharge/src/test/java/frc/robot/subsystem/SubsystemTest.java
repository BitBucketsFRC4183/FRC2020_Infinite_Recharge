package frc.robot.subsystem;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.anyString;
import static org.powermock.api.mockito.PowerMockito.mockStatic;
import static org.powermock.api.mockito.PowerMockito.when;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import org.junit.Before;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.core.classloader.annotations.SuppressStaticInitializationFor;
import org.powermock.modules.junit4.PowerMockRunner;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.PortsJNI;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SolenoidJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * To unit test subsystems we need to mock out a bunch of static initializers
 * and static classes so we can test only the logic of our subsystems
 */
@RunWith(PowerMockRunner.class)
@PrepareForTest({ NetworkTablesJNI.class, HAL.class, PortsJNI.class, DriverStation.class, HALUtil.class,
        ConstantsJNI.class, SolenoidJNI.class, SimDeviceJNI.class, MotControllerJNI.class, SmartDashboard.class,
        Preferences.class })
@SuppressStaticInitializationFor({ "edu.wpi.first.networktables.NetworkTablesJNI", "edu.wpi.first.hal.JNIWrapper",
        "com.ctre.phoenix.CTREJNIWrapper", "edu.wpi.first.wpilibj.DriverStation", 
        "edu.wpi.first.wpilibj.Preferences" })
public abstract class SubsystemTest {

    /**
     * Create a mock of the DriverStation so that it will not start a thread when
     * instantiated.
     */
    @Mock
    protected DriverStation driverStation;

    @Mock
    protected Preferences preferences;

    @Before
    public void beforeTest() throws Exception {
        // mock out all the static methods of these JNI classes
        // this makes all those native calls do nothing so we avoid exceptions
        mockStatic(NetworkTablesJNI.class);
        mockStatic(HAL.class);
        mockStatic(HALUtil.class);
        mockStatic(ConstantsJNI.class);
        mockStatic(PortsJNI.class);
        mockStatic(MotControllerJNI.class);
        mockStatic(SimDeviceJNI.class);

        // Mock the DriverStation class as well. We don't want it instantiated at all
        mockStatic(DriverStation.class);
        when(DriverStation.getInstance()).thenReturn(driverStation);

        // Mock the Preferences class as well, just like the DriverStation
        mockStatic(Preferences.class);
        when(Preferences.getInstance()).thenReturn(preferences);
        when(preferences.getString(anyString(), anyString())).thenReturn("");

        // make sure any solenoid creation attempts will go through. These methods are
        // called
        // before a solenoid is created to make sure the module and channel are actually
        // available
        // on the robot.
        mockStatic(SolenoidJNI.class);
        when(SolenoidJNI.checkSolenoidModule(anyInt())).thenReturn(true);
        when(SolenoidJNI.checkSolenoidChannel(anyInt())).thenReturn(true);

        // mock out the SmartDashboard as well
        mockStatic(SmartDashboard.class);
    }

}