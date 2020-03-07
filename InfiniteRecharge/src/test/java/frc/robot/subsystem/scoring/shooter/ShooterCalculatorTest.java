package frc.robot.subsystem.scoring.shooter;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.when;

import java.util.Arrays;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import frc.robot.subsystem.vision.VisionSubsystem;

/**
 * This will test the ShooterCalculator. It isn't a subsystem and doesn't load motors, so it's easy to test.
 * We only have to mock the VisionSubsystem to return distances and valid targets.
 */
@RunWith(MockitoJUnitRunner.class)
public class ShooterCalculatorTest {

    @Mock
    VisionSubsystem visionSubsystem;

    /**
     * Test the ShooterCalculator calculateHoodAngle_deg with no target
     * 
     * @throws Exception
     */
    @Test
    public void testCalculateHoodAngle_degNoTarget() throws Exception {
        ShooterCalculator shooterCalculator = new ShooterCalculator();
        shooterCalculator.initialize(visionSubsystem);

        // check no valid target
        when(visionSubsystem.getValidTarget()).thenReturn(false);
        assertEquals(ShooterConstants.DEFAULT_ELEVATION_TARGET_DEG, shooterCalculator.calculateHoodAngle_deg(), 0);
        assertEquals(ShooterConstants.DEFAULT_SHOOTER_VELOCITY_RPM, shooterCalculator.calculateSpeed_rpm(), .1);
    }

    /**
     * Test the ShooterCalculator calculateHoodAngle_deg with an exact target
     * 
     * @throws Exception
     */
    @Test
    public void testCalculateHoodAngle_degExactTarget() throws Exception {
        // create the shooter calculator with test points so we don't worry about changing our unit tests
        // when we have real data
        ShooterCalculator shooterCalculator = new ShooterCalculator(Arrays.asList(
            new ShooterCalculator.VelocityPoint(10, 1000, 10),
            new ShooterCalculator.VelocityPoint(20, 2000, 20)
        ));
        shooterCalculator.initialize(visionSubsystem);

        // check valid target with 10 degrees ty and 120 in (10 ft) away
        double ty = 0;
        double distance_in = 10;
        when(visionSubsystem.getValidTarget()).thenReturn(true);
        when(visionSubsystem.getTy()).thenReturn(ty);
        when(visionSubsystem.approximateDistanceFromTarget(eq(ty))).thenReturn(distance_in);

        // assert that we direct the hood to 10 degrees and 1000 RPMs when we are 10 inches away
        assertEquals(10.0, shooterCalculator.calculateHoodAngle_deg(), .1);
        assertEquals(1000.0, shooterCalculator.calculateSpeed_rpm(), .1);
    }

        /**
     * Test the ShooterCalculator calculateHoodAngle_deg with an intermediate target
     * 
     * @throws Exception
     */
    @Test
    public void testCalculateHoodAngle_degIntermediateTarget() throws Exception {
        // create the shooter calculator with test points so we don't worry about changing our unit tests
        // when we have real data
        ShooterCalculator shooterCalculator = new ShooterCalculator(Arrays.asList(
            new ShooterCalculator.VelocityPoint(10, 1000, 10),
            new ShooterCalculator.VelocityPoint(20, 2000, 20)
        ));
        shooterCalculator.initialize(visionSubsystem);

        // check valid target with 10 degrees ty and 120 in (10 ft) away
        double ty = 0;
        double distance_in = 15;
        when(visionSubsystem.getValidTarget()).thenReturn(true);
        when(visionSubsystem.getTy()).thenReturn(ty);
        when(visionSubsystem.approximateDistanceFromTarget(eq(ty))).thenReturn(distance_in);

        // assert that we direct the hood to 10 degrees and 1000 RPMs when we are 10 inches away
        assertEquals(15.0, shooterCalculator.calculateHoodAngle_deg(), .1);
        assertEquals(1500.0, shooterCalculator.calculateSpeed_rpm(), .1);
    }
}
