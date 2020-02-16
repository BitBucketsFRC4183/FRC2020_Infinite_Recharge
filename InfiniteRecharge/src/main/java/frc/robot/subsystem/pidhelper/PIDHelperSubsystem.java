package frc.robot.subsystem.pidhelper;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;



public class PIDHelperSubsystem extends BitBucketSubsystem {
    private final String MOTOR_ID = getName() + "/Motor ID";
    private final String MOTOR_TYPE = getName() + "/Controller type";

    private enum Controllers {
        NONE ("NONE SELECTED"),
        TALON_SRX ("Talon SRX"),
        TALON_FX ("Talon FX");

        private final String NAME;
        Controllers(String name) {
            NAME = name;
        }

        String getName() { return NAME; }
    };
    private final SendableChooser<Controllers> MOTOR_TYPE_CHOOSER;

    private final String SLOT = getName() + "PID_SLOT";
    private final String KP = getName() + "kP";
    private final String KI = getName() + "kI";
    private final String KD = getName() + "kD";
    private final String KF = getName() + "kF";

    private BaseTalon talon = new WPI_TalonSRX(0);

    private int lastTalonID = 0;



    public PIDHelperSubsystem(Config config) {
        super(config);

        setName("PID Helper");

        SmartDashboard.putNumber(MOTOR_ID, -1);

        MOTOR_TYPE_CHOOSER = new SendableChooser<Controllers>();
    }



    public void initialize() {
        super.initialize();

        SmartDashboard.putNumber(MOTOR_ID, -1);

        MOTOR_TYPE_CHOOSER.setDefaultOption(Controllers.NONE.getName(), Controllers.NONE);
        MOTOR_TYPE_CHOOSER.addOption(Controllers.TALON_SRX.getName(), Controllers.TALON_SRX);
        MOTOR_TYPE_CHOOSER.addOption(Controllers.TALON_FX.getName(), Controllers.TALON_FX);
        SmartDashboard.putData(MOTOR_TYPE, MOTOR_TYPE_CHOOSER);

        SmartDashboard.putNumber(SLOT, -1);
        SmartDashboard.putNumber(KP, 0);
        SmartDashboard.putNumber(KI, 0);
        SmartDashboard.putNumber(KD, 0);
        SmartDashboard.putNumber(KF, 0);
    }

	public void testInit() {

    }
	
	public void testPeriodic() {
        int id = (int) SmartDashboard.getNumber(MOTOR_ID, 0);

        if (id != lastTalonID) {
            lastTalonID = id;

            updateMotor();
        }
    }
	
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();
    }



    private void updateMotor() {
        //talon = 
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }
}