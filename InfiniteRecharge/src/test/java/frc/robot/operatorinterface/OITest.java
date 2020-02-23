package frc.robot.operatorinterface;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.modules.junit4.PowerMockRunner;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystem.SubsystemTest;

@RunWith(PowerMockRunner.class)
public class OITest extends SubsystemTest {

    @Test
    public void testSpeed() {
        OI oi = new OI();

        // test no input
        assertEquals(0.0, oi.speed(), 0);

        // test half moved joystick
        Joystick driverJoystick = spy(oi.driverControl);
        when(driverJoystick.getRawAxis(OI.DRIVE_SPEED_AXIS)).thenReturn(.5);
        assertEquals(-0.5, oi.speed(), 0);

        // test with invert button on
        when(driverJoystick.getRawAxis(OI.DRIVE_SPEED_AXIS)).thenReturn(.5);
        when(driverJoystick.getRawButton(OI.DRIVE_INVERT_BUTTON)).thenReturn(true);
        assertEquals(0.5, oi.speed(), 0);

    }

}