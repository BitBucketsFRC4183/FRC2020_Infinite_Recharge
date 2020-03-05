package frc.robot.subsystem.scoring.shooter.ball_management;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class BallManagementSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Variables
    
    float deltaTimeBMS = 0;
    float timer = 0;

    //////////////////////////////////////////////////////////////////////////////
    // Motors

    // Talons
    WPI_TalonSRX motor;

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public BallManagementSubsystem(Config config) {
        super(config);
        // TODO Auto-generated constructor stub
    }

    public void initialize() {
        super.initialize();
        motor = MotorUtils.makeSRX(config.ballManagement.spinner);
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
    }

    /**
     * 
     * @param rate How rapidly should we give balls to the feeder?
     */
    public void fire(float rate) {

        // timer = timer + deltaTimeBMS;
        
        // Because you can neva 'ave enuf dakka!
        SmartDashboard.putString(getName() + "/State",
                "DAKKADAKKADAKKADAKKADAKKDAKKADAKKADAKKADAKKADAKKADAKKADAKKDAKKADAKKADAKKADAKKADAKKADAKKADAKKDAKKADAKKADAKKADAKKADAKKADAKKADAKKDAKKADAKKA");
        // if (timer <= 0.5) {
            motor.set(ControlMode.PercentOutput, rate);
        // } else {
        //     motor.set(ControlMode.PercentOutput, 0);
        // }
        // if (timer >= 1) {
        //     timer = 0;
        // }
    }

    public void doNotFire() {
        motor.set(0);
        timer = 0;
        SmartDashboard.putString(getName() + "/State", "Doing nothing");
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable(){
        motor.set(0);
    }

    @Override
    public void listTalons() {
        talons.add(motor);
    }
}