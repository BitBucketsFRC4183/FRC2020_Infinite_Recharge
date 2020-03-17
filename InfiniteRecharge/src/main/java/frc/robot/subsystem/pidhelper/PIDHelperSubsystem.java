package frc.robot.subsystem.pidhelper;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.MotorConfig;
import frc.robot.config.MotorConfig.EncoderType;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.talonutils.MotorUtils;



// a subsystem we tried to make to help us tune PID from the Dashboard, didn't get done
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
    private SendableChooser<Controllers> controllerChooser;

    private enum Slots {
        VELOCITY ("Velocity (" + MotorUtils.velocitySlot + ")", MotorUtils.velocitySlot),
        POSITION ("Position (" + MotorUtils.positionSlot + ")", MotorUtils.positionSlot);

        private final String NAME;
        private final int SLOT;

        Slots(String name, int slot) {
            NAME = name;
            SLOT = slot;
        }

        String getName() { return NAME; }
        int getSlot() { return SLOT; }
    }

    private SendableChooser<Slots> slotChooser;
    private final String SLOT_CHOOSER_NAME = getName() + "/Slot";

    private final String KP = getName() + "/kP";
    private final String KI = getName() + "/kI";
    private final String KD = getName() + "/kD";
    private final String KF = getName() + "/kF";
    private final String I_ZONE = getName() + "/IZone";

    private SendableChooser<EncoderType> encoderChooser;
    private final String ENCODER_TYPE = getName() + "/Encoder type";

    private final String INVERTED = getName() + "/Inverted";

    private final String MM_ACC = getName() + "/MM acceleration";
    private final String MM_CRUISE = getName() + "/MM cruise velocity";

    private final String POSITION = getName() + "/Encoder position";
    private final String VELOCITY = getName() + "/Encoder velocity";
    private final String CLOSED_LOOP_ERROR = getName() + "/Closed loop error";

    private final String CONTROL_MODE = getName() + "/Control Mode";
    private final SendableChooser<ControlMode> CONTROL_MODE_CHOOSER;

    private final String INPUT = getName() + "/Input";

    private final String UPDATE = getName() + "/Update config";

    private BaseTalon talon = new WPI_TalonSRX(0);

    private int lastTalonID = 0;



    public PIDHelperSubsystem(Config config) {
        super(config);

        setName("PID Helper");

        SmartDashboard.putNumber(MOTOR_ID, -1);

        controllerChooser = getControllerChooser(Controllers.NONE);
        slotChooser = getSlotChooser(Slots.POSITION);
        encoderChooser = getEncoderTypeChooser(EncoderType.None);

        CONTROL_MODE_CHOOSER = new SendableChooser<ControlMode>();
    }



    public void initialize() {
        super.initialize();

        SmartDashboard.putNumber(MOTOR_ID, -1);

        SmartDashboard.putData(MOTOR_TYPE, controllerChooser);
        SmartDashboard.putData(SLOT_CHOOSER_NAME, slotChooser);
        SmartDashboard.putData(ENCODER_TYPE, encoderChooser);

        SmartDashboard.putNumber(KP, 0);
        SmartDashboard.putNumber(KI, 0);
        SmartDashboard.putNumber(KD, 0);
        SmartDashboard.putNumber(KF, 0);
        SmartDashboard.putNumber(I_ZONE, 0);

        SmartDashboard.putBoolean(INVERTED, false);
        SmartDashboard.putNumber(MM_ACC, 0);
        SmartDashboard.putNumber(MM_CRUISE, 0);

        CONTROL_MODE_CHOOSER.setDefaultOption("PercentOutput", ControlMode.PercentOutput);
        CONTROL_MODE_CHOOSER.addOption("Velocity", ControlMode.Velocity);
        CONTROL_MODE_CHOOSER.addOption("MotionMagic", ControlMode.MotionMagic);
        SmartDashboard.putData(CONTROL_MODE, CONTROL_MODE_CHOOSER);
    }

	public void testInit() {

    }
	
	public void testPeriodic() {
        int id = (int) SmartDashboard.getNumber(MOTOR_ID, 0);

        if (id != lastTalonID) {
            lastTalonID = id;

            updateMotor();

            SmartDashboard.putNumber(INPUT, 0);
        }

        if (SmartDashboard.getBoolean(UPDATE, false)) {
            update();
            
            SmartDashboard.putBoolean(UPDATE, false);
        }

        talon.set(CONTROL_MODE_CHOOSER.getSelected(), SmartDashboard.getNumber(INPUT, 0));
    }
	
	public void diagnosticsCheck() {

    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();
    }



    private void updateMotor() {
        Controllers controller = controllerChooser.getSelected();
        if (controller == Controllers.TALON_FX) {
            talon = new WPI_TalonFX(lastTalonID);
        } else if (controller == Controllers.TALON_SRX) {
            talon = new WPI_TalonSRX(lastTalonID);
        } else {
            talon = new WPI_TalonSRX(0);

            return;
        }

        updateDashboardConfig();
    }

    private void updateDashboardConfig() {
        int slot = slotChooser.getSelected().getSlot();



        FeedbackDevice sensor = FeedbackDevice.valueOf(talon.configGetParameter(ParamEnum.eFeedbackSensorType, slot));
        switch (sensor) {
            case QuadEncoder: {
                encoderChooser = getEncoderTypeChooser(EncoderType.Quadrature);
            }
            case CTRE_MagEncoder_Relative: {
                encoderChooser = getEncoderTypeChooser(EncoderType.Relative);
            }
            case CTRE_MagEncoder_Absolute: {
                encoderChooser = getEncoderTypeChooser(EncoderType.Absolute);
            }
            case IntegratedSensor: {
                encoderChooser = getEncoderTypeChooser(EncoderType.Integrated);
            }
            default: {
                encoderChooser = getEncoderTypeChooser(EncoderType.None);
            }
        }
        SmartDashboard.putData(ENCODER_TYPE, encoderChooser);



        SmartDashboard.putBoolean(INVERTED, talon.getInverted());
        // talon doesn't let you access the sensor phase :/

        SmartDashboard.putNumber(MM_ACC, talon.configGetParameter(ParamEnum.eMotMag_Accel, 0));
        SmartDashboard.putNumber(MM_CRUISE, talon.configGetParameter(ParamEnum.eMotMag_VelCruise, 0));

        SmartDashboard.putNumber(KP, talon.configGetParameter(ParamEnum.eProfileParamSlot_P, slot));
        SmartDashboard.putNumber(KI, talon.configGetParameter(ParamEnum.eProfileParamSlot_I, slot));
        SmartDashboard.putNumber(KD, talon.configGetParameter(ParamEnum.eProfileParamSlot_D, slot));
        SmartDashboard.putNumber(KF, talon.configGetParameter(ParamEnum.eProfileParamSlot_F, slot));
        SmartDashboard.putNumber(I_ZONE, talon.configGetParameter(ParamEnum.eProfileParamSlot_IZone, slot));
    }

    private void update() {
        MotorConfig config = new MotorConfig();
        
        config.encoderType = encoderChooser.getSelected();
        config.inverted = SmartDashboard.getBoolean(INVERTED, false);
        config.motionMagicAcceleration = (int) SmartDashboard.getNumber(MM_ACC, 0);
        config.motionMagicCruiseVelocity = (int) SmartDashboard.getNumber(MM_CRUISE, 0);
        config.positionPIDF = new PIDF(
            talon.configGetParameter(ParamEnum.eProfileParamSlot_P, MotorUtils.positionSlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_I, MotorUtils.positionSlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_D, MotorUtils.positionSlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_F, MotorUtils.positionSlot)
        );
        config.velocityPIDF = new PIDF(
            talon.configGetParameter(ParamEnum.eProfileParamSlot_P, MotorUtils.velocitySlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_I, MotorUtils.velocitySlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_D, MotorUtils.velocitySlot),
            talon.configGetParameter(ParamEnum.eProfileParamSlot_F, MotorUtils.velocitySlot)
        );
        config.id = lastTalonID;



        MotorUtils.motorInit(talon, config);
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        SmartDashboard.putNumber(POSITION, talon.getSelectedSensorPosition());
        SmartDashboard.putNumber(VELOCITY, talon.getSelectedSensorVelocity());
        SmartDashboard.putNumber(CLOSED_LOOP_ERROR, talon.getClosedLoopError());
    }






    private SendableChooser<Controllers> getControllerChooser(Controllers def) {
        Controllers[] values = Controllers.values();

        SendableChooser<Controllers> chooser = new SendableChooser<Controllers>();

        for (int i = 0; i < values.length; i++) {
            if (values[i] == def) {
                chooser.setDefaultOption(def.getName(), def);
            } else {
                chooser.addOption(values[i].getName(), values[i]);
            }
        }

        return chooser;
    }

    private SendableChooser<Slots> getSlotChooser(Slots def) {
        Slots[] values = Slots.values();

        SendableChooser<Slots> chooser = new SendableChooser<Slots>();

        for (int i = 0; i < values.length; i++) {
            if (values[i] == def) {
                chooser.setDefaultOption(def.getName(), def);
            } else {
                chooser.addOption(values[i].getName(), values[i]);
            }
        }

        return chooser;
    }

    private SendableChooser<EncoderType> getEncoderTypeChooser(EncoderType def) {
        EncoderType[] values = EncoderType.values();

        SendableChooser<EncoderType> chooser = new SendableChooser<EncoderType>();

        for (int i = 0; i < values.length; i++) {
            if (values[i] == def) {
                chooser.setDefaultOption(def.toString(), def);
            } else {
                chooser.addOption(values[i].toString(), values[i]);
            }
        }

        return chooser;
    }

    public void disable(){
    }

    @Override
    public void listTalons() {}
}