package frc.robot.utils.data.filters.statespace.kalman.filters;

import org.junit.Test;

import frc.robot.utils.data.filters.RunningAverageFilter;

import static org.junit.Assert.*;

public class RunningAverageFilterTest {

    @Test
    public void testCalculateSimple() {
        // it should always return 10 if we pass in 10
        RunningAverageFilter filter = new RunningAverageFilter(3);
        assertEquals(10.0, filter.calculate(10.0), 0);
        assertEquals(10.0, filter.calculate(10.0), 0);
        assertEquals(10.0, filter.calculate(10.0), 0);
        assertEquals(10.0, filter.calculate(10.0), 0);
    }

    @Test
    public void testCalculateAverage() {
        // it should always return 10 if we pass in 10
        RunningAverageFilter filter = new RunningAverageFilter(3);
        assertEquals(10.0, filter.calculate(10.0), 0);
        assertEquals(10.5, filter.calculate(11.0), 0);
        assertEquals(11.0, filter.calculate(12.0), 0);
        assertEquals(11.0, filter.calculate(10.0), 0);
    }

}
