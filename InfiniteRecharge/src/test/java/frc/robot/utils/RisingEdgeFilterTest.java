package frc.robot.utils;

import org.junit.Test;

import static org.junit.Assert.*;

public class RisingEdgeFilterTest {

    @Test
    public void calculate() {
        RisingEdgeFilter filter = new RisingEdgeFilter();
        assertEquals(false, filter.calculate(false));

        // first time true, second time false until another true
        assertEquals(true, filter.calculate(true));
        assertEquals(false, filter.calculate(true));
        assertEquals(false, filter.calculate(true));

        // After encountering another false, it should return true
        assertEquals(false, filter.calculate(false));
        assertEquals(true, filter.calculate(true));
    }
}
