package frc.robot.subsystem.scoring.shooter;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.anyFloat;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.powermock.api.mockito.PowerMockito.whenNew;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.subsystem.SubsystemTest;
import frc.robot.subsystem.scoring.shooter.ball_management.BallManagementSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

/**
 * This will test the ShooterSubsystem We prepare a couple other classes for test
 * because this subsystem creates a motor and a solenoid
 */
@RunWith(PowerMockRunner.class)
@PrepareForTest({ 
    // need this to mock whenNew for motors. You have to prepare the class that
    // calls new for whenNew to work
    MotorUtils.class, 
    // we need to prepare the ShooterSubsystem class for mocking because we mock the whenNew() for the BMS.
    ShooterSubsystem.class
})
public class ShooterSubsystemTest extends SubsystemTest {

    /**
     * This is a mock version of the motors the shooter subsystem uses
     */
    @Mock
    WPI_TalonSRX azimuthMotor;

    @Mock
    WPI_TalonSRX elevationMotor;

    @Mock
    WPI_TalonSRX ballPropulsionMotor;

    @Mock
    WPI_TalonSRX feeder;

    @Mock
    VisionSubsystem visionSubsystem;

    @Mock
    BallManagementSubsystem ballManagementSubsystem;

    /**
     * We use the config in this test, so get a copy of it
     */
    Config config;

    @Before
    public void beforeTest() throws Exception {
        super.beforeTest();

        config = ConfigChooser.getConfig();

        // when our subsystem is initialized, it will create a motor and a solenoid. We
        // don't want
        // actual motors and solenoids to be created, we want mock ones. Make sure we
        // return our mocked
        // instances instead of new real instances.
        whenNew(WPI_TalonSRX.class).withArguments(eq(config.shooter.azimuth.id)).thenReturn(azimuthMotor);
        whenNew(WPI_TalonSRX.class).withArguments(eq(config.shooter.elevation.id)).thenReturn(elevationMotor);
        whenNew(WPI_TalonSRX.class).withArguments(eq(config.shooter.shooter.id)).thenReturn(ballPropulsionMotor);
        whenNew(WPI_TalonSRX.class).withArguments(eq(config.shooter.feeder.id)).thenReturn(feeder);

        // mock out the BMS 
        whenNew(BallManagementSubsystem.class).withAnyArguments().thenReturn(ballManagementSubsystem);
    }

    /**
     * Test the ShooterSubsystem spinBMS function
     * 
     * @throws Exception
     */
    @Test
    public void testSpinBMS() throws Exception {
        config.enableBallManagementSubsystem = true;
        // create and initialize the subsystem so we have motor objects
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem(config, visionSubsystem);
        shooterSubsystem.initialize();

        // call spinBMS
        shooterSubsystem.spinBMS();

        // verify that the we told the ballManagementSubsystem to fire
        verify(ballManagementSubsystem).fire(anyFloat());
    }

}
