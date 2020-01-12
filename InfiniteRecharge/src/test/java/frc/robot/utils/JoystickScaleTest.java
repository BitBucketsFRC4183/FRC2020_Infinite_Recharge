package frc.robot.utils;

import org.junit.Test;

import static org.junit.Assert.*;

public class JoystickScaleTest {

    @Test
    public void map() {
        JoystickScale scale = JoystickScale.LINEAR;

        // if our input is -2 to 2, but we want to map it to -1 to 1, the map function should
        // map 2 to 1, 0 to 0, 1 to .5, and -1 to -.5
        assertEquals(1, scale.map(2, -2, 2, -1, 1), 0);
        assertEquals(0, scale.map(0, -2, 2, -1, 1), 0);
        assertEquals(.5, scale.map(1, -2, 2, -1, 1), 0);
        assertEquals(-.5, scale.map(-1, -2, 2, -1, 1), 0);
    }

    @Test
    public void rescale() {
        assertEquals(2, JoystickScale.LINEAR.rescale(2), 0);
        assertEquals(4, JoystickScale.SQUARE.rescale(2), 0);
        assertEquals(8, JoystickScale.CUBE.rescale(2), 0);

        // the deadband should record .1 as 0
        assertEquals(0, JoystickScale.LINEAR.rescale(.1, .1), 0);

    }
}
