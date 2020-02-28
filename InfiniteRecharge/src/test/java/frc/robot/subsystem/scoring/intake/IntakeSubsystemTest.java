package frc.robot.subsystem.scoring.intake;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.powermock.api.mockito.PowerMockito.mockStatic;
import static org.powermock.api.mockito.PowerMockito.when;
import static org.powermock.api.mockito.PowerMockito.whenNew;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.ConfigChooser;
import frc.robot.subsystem.SubsystemTest;
import frc.robot.utils.talonutils.MotorUtils;

/**
 * This will test the IntakeSubsystem We prepare a couple other classes for test
 * because this subsystem creates a motor and a solenoid
 */
@RunWith(PowerMockRunner.class)
@PrepareForTest({ MotorUtils.class, // need this to mock whenNew for motors. YOu have to prepare the class that
                                    // calls new for whenNew to work
        IntakeSubsystem.class // we also mock new creations of the DoubleSolenoid
})
public class IntakeSubsystemTest extends SubsystemTest {

    /**
     * This is a mock version of the motor the intake subsystem uses
     */
    @Mock
    WPI_TalonSRX motor;

    /**
     * This is a mock version of the solenoid the intake subsystem uses
     */
    @Mock
    DoubleSolenoid intakePivet;

    /**
     * We use the config in this test, so get a copy of it
     */
    Config config;

    @Before
    public void beforeTest() throws Exception {
        super.beforeTest();
        config = ConfigChooser.getConfig();
        config.intake.intakePivotEnabled = true;

        // when our subsystem is initialized, it will create a motor and a solenoid. We
        // don't want
        // actual motors and solenoids to be created, we want mock ones. Make sure we
        // return our mocked
        // instances instead of new real instances.
        whenNew(WPI_TalonSRX.class).withArguments(eq(config.intake.intake.id)).thenReturn(motor);
        whenNew(DoubleSolenoid.class).withArguments(anyInt(), anyInt()).thenReturn(intakePivet);
    }

    /**
     * Test the IntakeSubsystem off + periodic function
     * 
     * @throws Exception
     */
    @Test
    public void testOff() throws Exception {
        // create and initialize the subsystem so we have motor objects
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // call off
        intakeSubsystem.off();
        intakeSubsystem.periodic(.02f);

        // verify that the motor is set to intake
        verify(motor).set(eq(0.0));
    }

    /**
     * Test the IntakeSubsystem intaking + periodic function
     * 
     * @throws Exception
     */
    @Test
    public void testIntaking() throws Exception {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // call intake
        mockStatic(SmartDashboard.class);
        when(SmartDashboard.getNumber(eq("IntakeSubsystem/Intake Speed"), anyDouble())).thenReturn(.1);
        intakeSubsystem.intake();
        intakeSubsystem.periodic(.02f);

        // verify that the motor is set to intake
        verify(motor).set(eq(.1));
    }

    /**
     * Test the IntakeSubsystem outtaking + periodic function
     * 
     * @throws Exception
     */
    @Test
    public void testOuttaking() throws Exception {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // call outtake
        mockStatic(SmartDashboard.class);
        when(SmartDashboard.getNumber(eq("IntakeSubsystem/Intake Speed"), anyDouble())).thenReturn(.1);
        intakeSubsystem.outake();
        intakeSubsystem.periodic(.02f);

        // verify that the motor is set to intake
        verify(motor).set(eq(-.1));
    }

    /**
     * Test the IntakeSubsystem intakePivet toggle function
     * 
     * @throws Exception
     */
    @Test
    public void testToggleIntakeArm1() throws Exception {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // verify if it's forward, we call reverse
        when(intakePivet.get()).thenReturn(Value.kForward);
        intakeSubsystem.toggleIntakeArm();
        verify(intakePivet, times(1)).set(eq(Value.kReverse));
    }

    /**
     * Test the IntakeSubsystem intakePivet toggle function
     * 
     * @throws Exception
     */
    @Test
    public void testToggleIntakeArm2() throws Exception {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // verify if it's reverse, we call forward
        when(intakePivet.get()).thenReturn(Value.kReverse);
        intakeSubsystem.toggleIntakeArm();
        // intakePivet is actually set to forward twice, once at startup and once in
        // this method
        verify(intakePivet, times(2)).set(eq(Value.kForward));
    }

    /**
     * Test the IntakeSubsystem intakePivet toggle function
     * 
     * @throws Exception
     */
    @Test
    public void testToggleIntakeArm3() throws Exception {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(config);
        intakeSubsystem.initialize();

        // verify if it's off, we call forward
        when(intakePivet.get()).thenReturn(Value.kOff);
        intakeSubsystem.toggleIntakeArm();
        // intakePivet is actually set to forward twice, once at startup and once in
        // this method
        verify(intakePivet, times(2)).set(eq(Value.kForward));
    }

}
