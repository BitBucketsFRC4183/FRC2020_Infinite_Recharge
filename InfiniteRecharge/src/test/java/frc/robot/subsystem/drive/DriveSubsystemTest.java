package frc.robot.subsystem.drive;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.when;
import static org.powermock.api.mockito.PowerMockito.whenNew;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.SubsystemTest;
import frc.robot.subsystem.navigation.NavigationSubsystem;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

/**
 * This will test the DriveSubsystem We prepare a couple other classes for test
 * because this subsystem creates a motor and a solenoid
 */
@RunWith(PowerMockRunner.class)
@PrepareForTest({ 
    // need this to mock whenNew for motors. You have to prepare the class that
    // calls new for whenNew to work
    MotorUtils.class, 
    // we need to prepare the DriveSubsystem class for mocking because we mock the whenNew() for the BMS.
    DriveSubsystem.class
})
public class DriveSubsystemTest extends SubsystemTest {

    /**
     * This is a mock version of the motors the drive subsystem uses
     */
    @Mock
    WPI_TalonFX leftMotorLeader;
    
    @Mock
    WPI_TalonFX leftMotorFollower;
    
    @Mock
    WPI_TalonFX rightMotorLeader;

    @Mock
    WPI_TalonFX rightMotorFollower;

    @Mock
    NavigationSubsystem navigationSubsystem;

    @Mock
    OI oi;

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
        whenNew(WPI_TalonFX.class).withArguments(eq(config.drive.leftIDs[0])).thenReturn(leftMotorLeader);
        whenNew(WPI_TalonFX.class).withArguments(eq(config.drive.leftIDs[1])).thenReturn(leftMotorFollower);
        whenNew(WPI_TalonFX.class).withArguments(eq(config.drive.rightIDs[0])).thenReturn(rightMotorLeader);
        whenNew(WPI_TalonFX.class).withArguments(eq(config.drive.rightIDs[1])).thenReturn(rightMotorFollower);

    }

    /**
     * Test the DriveSubsystem testGetLeftDistance_meters function
     * 
     * @throws Exception
     */
    @Test
    public void testGetLeftDistance_meters() throws Exception {
        // create and initialize the subsystem so we have motor objects
        DriveSubsystem driveSubsystem = new DriveSubsystem(config, navigationSubsystem, new VisionSubsystem(config));
        driveSubsystem.initialize();

        // verify that spinning the wheel fully around once gives us a left distance of 6 inch wheel
        // a full wheel ritation is 2048 ticks 10.88 times because of the drive gear ratio
        when(leftMotorLeader.getSelectedSensorPosition()).thenReturn((int)(2048 * config.drive.gearRatio));
        assertEquals(config.drive.wheelRadius_in * 2 * Math.PI * .0254, driveSubsystem.getLeftDistance_meters(), .01);
    }

    /**
     * Test the DriveSubsystem testGetLeftDistance_meters function
     * 
     * @throws Exception
     */
    @Test
    public void testGetLeftVelocity_mps() throws Exception {
        // create and initialize the subsystem so we have motor objects
        DriveSubsystem driveSubsystem = new DriveSubsystem(config, navigationSubsystem, new VisionSubsystem(config));
        driveSubsystem.initialize();

        // verify that spinning the wheel fully around once per 100ms gives us 
        // a speed of 10 full rotations every second
        when(leftMotorLeader.getSelectedSensorVelocity()).thenReturn((int)(2048 * config.drive.gearRatio));
        assertEquals(config.drive.wheelRadius_in * 2 * Math.PI * .0254 * 10, driveSubsystem.getLeftVelocity_mps(), .01);
    }

    /**
     * This is a sanity test for the Odemetry class, just to show how it works
     */
    @Test
    public void testOdometry() throws Exception {
        DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(90), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)));

        assertEquals(0, odometry.getPoseMeters().getTranslation().getX(), 0);
        assertEquals(0, odometry.getPoseMeters().getTranslation().getX(), 0);

        // go straight 1 meter on the y-axis
        odometry.update(Rotation2d.fromDegrees(90), 1, 1);
        assertEquals(0, odometry.getPoseMeters().getTranslation().getX(), 0.01);
        assertEquals(1, odometry.getPoseMeters().getTranslation().getY(), 0.01);

        // go straight another meter on the y-axis
        odometry.update(Rotation2d.fromDegrees(90), 2, 2);
        assertEquals(0, odometry.getPoseMeters().getTranslation().getX(), 0.01);
        assertEquals(2, odometry.getPoseMeters().getTranslation().getY(), 0.01);

    }

}
