package frc.robot.subsystem.scoring.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.operatorinterface.OI;
import frc.robot.operatorinterface.PS4Constants;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class ShooterSubsystem extends BitBucketSubsystem {

    //////////////////////////////////////////////////////////////////////////////
    // Booleans
    public boolean shooting = false;

    //////////////////////////////////////////////////////////////////////////////
    // Talons
    protected WPI_TalonSRX azimuthMotor;

    //////////////////////////////////////////////////////////////////////////////
    // Neos
    protected CANSparkMax ballPropulsionMotor;

    //////////////////////////////////////////////////////////////////////////////
    // Methods

    public ShooterSubsystem(Config config) {
        super(config);
    }

    @Override
    public void initialize() {
        azimuthMotor = new WPI_TalonSRX(config.shooter.azimuth.id);
        ballPropulsionMotor = new CANSparkMax(config.shooter.shooter.id, CANSparkMaxLowLevel.MotorType.kBrushless);

        MotorUtils.motorInit(azimuthMotor, config.shooter.azimuth);
        MotorUtils.motorInit(ballPropulsionMotor, config.shooter.shooter);
    }

    @Override
    public void diagnosticsInitialize() {
        // TODO Auto-generated method stub

    }

    @Override
    public void diagnosticsPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void diagnosticsCheck() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
         if(shooting){
            ballPropulsionMotor.set(SmartDashboard.getNumber(getName() + "/Shooter Velocity", 0.2));
            
        } else {
            ballPropulsionMotor.set(0);
        }
    }
    public void shoot() {
        shooting = true;
    }
    public void doNotShoot() {
        shooting = false;
    }
    public void rotate(double spinRate) {
        
        double finalSpinRate = spinRate * SmartDashboard.getNumber(getName() + "/Turret Speed Multiplier", 0.1);
        
        azimuthMotor.set(finalSpinRate);

    }

}