package frc.robot.subsystem.drive;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.powermock.modules.junit4.PowerMockRunner;

import frc.robot.config.Config;
import frc.robot.subsystem.SubsystemTest;

@RunWith(PowerMockRunner.class)
public class DriveUtilsTest extends SubsystemTest {

    @Test
    public void testIpsToTicksP100() {
        Config config = new Config();
        config.drive.wheelRadius_in = 3;
        config.drive.ticksPerRevolution = 2048;

        double circumference = config.drive.wheelRadius_in * 2 * Math.PI;

        DriveUtils driveUtils = new DriveUtils(config);

        // get ticks per 100ms for 10 full wheel rotations, should be 2048 ticks per
        // 100ms
        assertEquals(2048, driveUtils.ipsToTicksP100(10 * circumference), 0);
    }

    @Test
    public void testTicksP100ToIps() {
        Config config = new Config();
        config.drive.wheelRadius_in = 3;
        config.drive.ticksPerRevolution = 2048;

        double circumference = config.drive.wheelRadius_in * 2 * Math.PI;

        DriveUtils driveUtils = new DriveUtils(config);

        // 2048 ticks per 100ms is equivalent of 10 rotations of a 6 inch wheel in one second
        assertEquals(10 * circumference, driveUtils.ticksP100ToIps(2048), 0);
    }    
}
