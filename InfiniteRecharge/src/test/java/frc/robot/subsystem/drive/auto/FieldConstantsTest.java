package frc.robot.subsystem.drive.auto;

import java.util.List;

import org.junit.Test;

import static org.junit.Assert.*;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystem.drive.auto.FieldConstants;



public class FieldConstantsTest {
    @Test
    public void testRelativeTransform() {
        Translation2d pos = FieldConstants.transformToRobot(FieldConstants.OUR_POWER_CELL_1, FieldConstants.START_CENTER_POWER_PORT);

        // assertEquals(pos.getX(), 3.114802, 0.00001);
        // assertEquals(pos.getY(), 1.699514, 0.00001);
    }
}