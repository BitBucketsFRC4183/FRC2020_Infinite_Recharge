package frc.robot.utils.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class MathUtilsTest {

    @Test
    public void testUnitConverter() {
        // convert 6000 rpm to ticks per 100ms
        // from the math, 6000 RPMs should come out to 20480 ticks per 100ms
        // assuming 2048 ticks per revolution
        assertEquals(20480, MathUtils.unitConverter(6000, 600, 2048), 0);

        // test converting degrees to ticks assuming 4096 ticks per revolution
        // 90 degrees is 1/4 of a rotation, or 1024 ticks
        assertEquals(1024, MathUtils.unitConverter(90, 360, 4096), 0);
    }

}
