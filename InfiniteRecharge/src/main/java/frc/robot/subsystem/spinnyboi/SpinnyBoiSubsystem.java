package frc.robot.subsystem.spinnyboi;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;



public class SpinnyBoiSubsystem extends BitBucketSubsystem {
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
    WPI_TalonSRX motor;
    
    public SpinnyBoiSubsystem(Config config) {
        super(config);
    }

    public void initialize() {
        super.initialize();
        motor = MotorUtils.makeSRX(config.spinnyboi.spinner);
    }

	public void testInit() {

    }
	
	public void testPeriodic() {

    }
	
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();

        Color detectedColor = m_colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = m_colorSensor.getIR();
    
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
    
        /**
         * In addition to RGB IR values, the color sensor can also return an 
         * infrared proximity value. The chip contains an IR led which will emit
         * IR pulses and measure the intensity of the return. When an object is 
         * close the value of the proximity will be large (max 2047 with default
         * settings) and will approach zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object
         * or provide a threshold for when an object is close enough to provide
         * accurate color values.
         */
        int proximity = m_colorSensor.getProximity();
    
        SmartDashboard.putNumber("Proximity", proximity);
    
    }

    public void rotationControl(){
        //Rotate the wheel 3 - 5 times

        motor.set(ControlMode.MotionMagic, SpinnyBoiConstants.SPIN_NUM);
    }

    public void manualRotationControl() {
        // In case something goes horribly wrong

        // motor.set()
    }

    public void colorControl(){
        //Rotate the wheel to the color specified by the FMS
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
        motor.set(0);
    }

    @Override
    protected void listTalons() {
        talons.add(motor);
    }
}