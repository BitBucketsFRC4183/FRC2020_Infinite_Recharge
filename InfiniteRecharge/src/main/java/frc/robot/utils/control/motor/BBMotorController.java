package frc.robot.utils.control.motor;



import frc.robot.utils.data.DataWindow;

import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.motionprofile.motionmagic.MotionMagic;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.control.MotorInfo;
import frc.robot.utils.control.MotionConfig;

import frc.robot.utils.control.encoder.*;

import frc.robot.utils.math.units.BaseUnit;
import frc.robot.utils.math.units.Unit;
import frc.robot.utils.math.units.Units;
import frc.robot.utils.math.units.BaseUnit.Dimension;
import frc.robot.utils.math.units.UnitBuilder;
import frc.robot.utils.math.units.Quantity;

import java.util.ArrayList;
import java.util.HashMap;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file and
 * everything in this package. This
 * was an attempt to consolidate
 * different APIs used for different
 * motor controller types, but it
 * never got tested
 * ==================================
 */
/**
 * To generalize to different types of motor controllers (TalonSRX/SparkMax as of right now)
 * that all have different method names/etc, it's useful to have an abstract class for a generic
 * motor controller (Bit Bucket Motor Controller). It's annoying enough to memorize the
 * oddly specific names some APIs (*cough* CTRE *cough*) gives to their methods so a convenient
 * way to circumvent this is a standardized set of methods for all motor controllers, intuitively
 * named
 * 
 * We use an abstract class instead of an interface because it allows us to have standardized
 * methods for all sorts of motor controllers that are not provided/may not be "standard"
 * in motor controller APIs we use. The subclasses themselves don't need to extend their
 * respective motor controller since they can just store an instance of such.
 * 
 * We can implement any methods in subclasses such as BBSparkMax and BBTalonSRX.
 * These wrapper classes contain their corresponding motor controller and by use of
 * standardized methods in BBMotorController can give other classes information about
 * or let other classes control the inside motor.
 * 
 * Just makes the code nicer to write and shorter (CTRE is very verbose) and hopefully
 * easier to understand, which is always good.
 * 
 * Generalizing also allows us to generalize in our control methods. Do we want to use
 * provided functional PID(F) or do we want to experiment with state-space?
 */
public abstract class BBMotorController {
    private final int DEVICE_ID;
    public BBMotorController(int deviceID) {
        DEVICE_ID = deviceID;
    }



    public int getDeviceID() {
        return DEVICE_ID;
    }




    /*
     * NOTE: traditionally, you put all fields and enums before all methods.
     * This will not be the case in this class - it is written in a way that makes sense
     * as you read from top to bottom. Fields will be declared as needed more the methods
     * that follow
     */



    /** MotionMagic parameters to be used */
    protected MotionMagic motionMagic;





    /*
     * While TalonSRX reserves 2 slots for PIDF constants (0 and 1), SparkMax does not
     * so we cannot generally assume this functionality will be provided and should thus
     * implement it ourselves.
     */
    /**
     * ArrayList of MotionConfigs that are stored in our motor controller wrapper objects.
     * Internally, TalonSRXs can store at most 2 and SparkMaxes can store 4, but we can store
     * more in motor controller objects in our code so long as we can efficiently communicate
     * when to switch to what configuration to the provided CTRE/RevRobotics/...
     * motor controller objects.
     */
    protected ArrayList<MotionConfig> motionConfigs = new ArrayList<MotionConfig>();
    /**
     * To facilitate keeping track of which MotionConfigs correspond to what "slot" in
     * our wrapper class (which then loosely corresponds to a slot in the motor controller if using
     * those), we can give names to each slot.
     * 
     * If no name is provided, will default to "Config[NUMBER]" where [NUMBER] is the number
     * of the (internal) slot as its added (first -> 0, second -> 1, ...)
     */
    protected HashMap<String, Integer> motionConfigNames = new HashMap<String, Integer>();

    /**
     * Get the number of available motion configurations in wraooed motor controller
     * 
     * @return number of available motion configurations (slots)
     */
    protected abstract int getMaxMotionSlots();
    /** number of available motion configurations */
    protected final int MOTION_SLOTS = getMaxMotionSlots();
    /** Array of motion config IDs in slots that are currently loaded into motor controller */
    protected int[] loadedMotionConfigs = new int[MOTION_SLOTS];
    /**
     * Number of motion configuration loaded in motor controller.
     * Not equal to loadedMotionConfigs.length in general: this would be so only if every
     * available slot was in use
     */
    protected int slotsUsed = 0;



    protected abstract void clearPIDF(int slot);
    protected abstract void clearMotionMagic(int slot);
    
    protected void clearMotionSlot(int slot) {
    	clearPIDF(slot);
    	clearMotionMagic(slot);
    }
    
    protected abstract void loadPID(PID constants, int slot);
    protected abstract void loadPIDF(PIDF constants, int slot);
    
    /** Configure MotionMagic parameters to use with native units */
    protected abstract void loadMotionMagic(double acc, double vel, int slot);

    protected void loadMotionMagic(MotionMagic mm, int slot) {
        Quantity vel = mm.getCruiseVelocity();
        Quantity acc = mm.getAcceleration();

        double vel_nu;
        double acc_nu;

        if (vel.getUnit().isCompatible(OMEGA_UNIT_NU)) {
            vel_nu = vel.to(OMEGA_UNIT_NU).getValue();
        } else {
            vel_nu = toAngular(vel).to(OMEGA_UNIT_NU).getValue();
        }

        if (acc.getUnit().isCompatible(ALPHA_UNIT_NU)) {
            acc_nu = acc.to(ALPHA_UNIT_NU).getValue();
        } else {
            acc_nu = toAngular(acc).to(ALPHA_UNIT_NU).getValue();
        }

        loadMotionMagic(acc_nu, vel_nu, slot);

        motionMagic = mm;
    }
    
    /**
     * Select a motion config loaded into the motor controller for use
     * 
     * @param slot number of the slot to be selected
     */
    public abstract void selectMotionConfigSlot(int slot);
    /**
     * Load in a motion slot stored in the BBMotorController into the
     * wrapped motor controller object.
     * 
     * @param configID id of motion config in motionConfigs
     * @param slot slot to load into
     */
    protected int loadMotionConfig(int configID, int slot) {
    	MotionConfig config = motionConfigs.get(configID);
    	
    	if (slotsUsed == MOTION_SLOTS) {
    		clearMotionSlot(slot);
    	}
    	
    	PID pid = config.getPID();
    	if (pid != null) {
    		if (pid instanceof PIDF) {
    			loadPIDF((PIDF) pid, slot);
    		} else {
    			loadPID(pid, slot);
    		}
    	}
    	
    	MotionMagic mm = config.getMotionMagic();
    	if (mm != null) {
    		loadMotionMagic(mm, slot);
    	}
    	
    	loadedMotionConfigs[slot] = configID; // very important
    	if (slot == slotsUsed) {
    		slotsUsed++;
    	}

        // "use" this new configuration
        motionConfigs.get(configID).use();
    	
    	// no need to load control type, that is to help find
    	// an appropriate motion configuration if necessary
    	
    	return slot;
    }
    /**
     * Load in a motion configuration stored in the BBMotorController into the
     * wrapped motor controller object. Defaults to the least used slot
     * in terms of amount of times its been reloaded into the motor controller
     * (probably not the best way but also how would you get to this point?
     * (also very overkill)).
     * 
     * @param configID id of motion configuration in motionConfigs
     * 
     * @return slot it was loaded into
     */
    protected int loadMotionConfig(int configID) {
        // which slot to laod into
        int slot;

        // if no slots are unused, look for least used slot to swap in a new set of constants
        // also should be at max == but just in case...
        if (slotsUsed >= MOTION_SLOTS) {
            // to find the minimum uses and corresponding slot, first assume last one
            int minUses = motionConfigs.get(loadedMotionConfigs[loadedMotionConfigs.length - 1]).getUses();
            slot = loadedMotionConfigs.length - 1;

            // loop through all slots in reverse order (except for last one, already covered)
            // to try to find a smaller amount of uses
            for (int i = loadedMotionConfigs.length - 2; i >= 0; i--) {
                // number of uses for this motion configuration
                int uses = motionConfigs.get(loadedMotionConfigs[i]).getUses();

                // test if its a new minimum
                if (uses < minUses) {
                    minUses = uses;
                    // set the new slot to switch into
                    slot = i;
                }
            }
        } else {
            // if haven't used all available slots, might as well load it into the motor controller
            // in the next unused slot
            slot = slotsUsed;
        }

        // load it in now that you know which slot to use
        return loadMotionConfig(configID, slot);
    }
// haha funny FRC number 254

    /**
     * Find a motion configuration by provided control type
     * 
     * @param controlType control type the motion configuration provides
     * 
     * @return location of possible motion configuration to use in motionConfigs or -1 if none found
     */
    public int findMotionConfig(ControlType controlType) {
        // loop through configs to look for a matching control type
        for (int i = 0; i < motionConfigs.size(); i++) {
            // test for matching control type of this set of PID constants
            if (motionConfigs.get(i).getControlType() == controlType) {
                return i;
            }
        }

        // default to -1 if none found
        return -1;
    }

    /**
     * Set the used motion configuration by position in motionConfigs
     * 
     * @param configID index of motion configuration to use in motionConfigs
     * 
     * @return slot slot the configuration was loaded into in motor controller
     */
    public int setMotionConfig(int configID) {
        // test if the configuration is already loaded, and if it is, select it
        for (int i = 0; i < slotsUsed; i++) {
            if (loadedMotionConfigs[i] == configID) {
                selectMotionConfigSlot(i);

                return i;
            }
        }

        // load the configuration in and get the slot to which it was loaded
        int slot = loadMotionConfig(configID);
        // select the slot it was loaded into
        selectMotionConfigSlot(slot);

        return slot;
    }

    /**
     * Set the used motion configuration by name, or do nothing if
     * not a recognized name.
     * 
     * @param configName name of motion configuration to use
     */
    public void setMotionConfig(String configName) {
        // make sure it is a recognized name first before setting it
        if (motionConfigNames.containsKey(configName)) {
            setMotionConfig(motionConfigNames.get(configName));
        }
    }

    /**
     * Set the used motion configuration by control type, or do nothing if
     * no matching control type is available
     * 
     * @param controlType control type to be provided by the motion configuration
     */
    public void setMotionConfig(ControlType controlType) {
        // try to find a set of corresponding configuration
        int configID = findMotionConfig(controlType);

        // if no configuration with the desired control type found, do nothing
        if (configID == -1) { return; } // rip
        // set the PID to the found ID
        setMotionConfig(configID);
    }



    /**
     * Add a motion configuration to the motor controller's stored set
     * 
     * @param config motion configuration to add to set of internally stored configurations
     * 
     * @return numeric ID of this configuration
     */
    public int addMotionConfiguration(MotionConfig config) {
        // get the ID to use in motionConfigs
        int id = motionConfigs.size();
        // give it a default name
        String name = "Config" + id;

        // add the motion configuration with the default name
        return addMotionConfiguration(config, name);
    }

    /**
     * Add a motion configuration to the motor controller's stored set,
     * indexed by a name
     * 
     * @param config configuration to add to set of internally stored configurations
     * @param name name of this configuration
     * 
     * @return numeric ID of this configuration
     */
    public int addMotionConfiguration(MotionConfig config, String name) {
        // get the ID to use in motionConfigs
        int id = motionConfigs.size();

        // add the configuration to the stored ArrayList
        motionConfigs.add(config);
        // name corresponds to id-th set in pidConstants
        motionConfigNames.put(name, id);

        // load in the first few until no more unused slots
        if (slotsUsed < MOTION_SLOTS) {
            loadMotionConfig(id, slotsUsed);
        }

        // return the ID
        return id;
    }





    protected abstract BaseUnit getThetaUnit_nu();
    protected BaseUnit THETA_UNIT_NU; // native encoder position units

    protected abstract BaseUnit getTimeUnit_nu();
    protected final BaseUnit TIME_UNIT_NU = getTimeUnit_nu();
    protected abstract BaseUnit getSecondTimeUnit_nu();
    protected final BaseUnit SECOND_TIME_UNIT_NU = getSecondTimeUnit_nu();

    private Unit OMEGA_UNIT_NU;
    private Unit ALPHA_UNIT_NU;

    protected void updateUnits_nu() {
        THETA_UNIT_NU = getThetaUnit_nu();
        OMEGA_UNIT_NU = (new UnitBuilder()).num(THETA_UNIT_NU).denom(TIME_UNIT_NU).make();
        ALPHA_UNIT_NU = (new UnitBuilder()).num(THETA_UNIT_NU).denom(TIME_UNIT_NU, SECOND_TIME_UNIT_NU).make();
    }





    protected BaseUnit THETA_UNIT_PU = Units.RAD; // preferred unit for input rotations
    protected BaseUnit TIME_UNIT_PU = Units.S;
    protected BaseUnit SECOND_TIME_UNIT_PU = Units.S;

    private enum PositionMeasurement {
        Angle,
        Distance
    }

    private PositionMeasurement positionMeasurement = PositionMeasurement.Angle;

    protected BaseUnit LENGTH_UNIT_PU = Units.FT;
    protected Quantity radius;

    protected Unit OMEGA_UNIT_PU = Units.RAD_PER_S;
    protected Unit ALPHA_UNIT_PU = Units.RAD_PER_S2;
    protected Unit RAD_PER_TIME = Units.RAD_PER_S;
    protected Unit RAD_PER_TIME2 = Units.RAD_PER_S2;
    protected Unit VEL_UNIT_PU = Units.FT_PER_S;
    protected Unit ACC_UNIT_PU = Units.FT_PER_S2;

    public void setRadius(double radius) {
        if (LENGTH_UNIT_PU == null) {
            return;
        }

        this.radius = new Quantity(radius, LENGTH_UNIT_PU);
    }

    public void setRadius(Quantity radius) {
        if (radius.getUnit().isCompatible(Dimension.Length)) {
            this.radius = radius;
        }
    }

    public void setMeasurementToDistance() {
        // needs to have a radius first
        if (radius == null) { return; }

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToDistance(double radius) {
        // need to have a unit
        if (LENGTH_UNIT_PU == null) {
            return;
        }

        this.radius = new Quantity(radius, LENGTH_UNIT_PU);

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToDistance(Quantity radius) {
    	if (!radius.getUnit().isCompatible(Dimension.Length)) {
    		return;
    	}
    	
        this.radius = radius;
        
        if (LENGTH_UNIT_PU == null) {
        	// default to radius unit
        	LENGTH_UNIT_PU = radius.getUnit().getNumeratorList().get(0);
        }

        positionMeasurement = PositionMeasurement.Distance;
    }

    public void setMeasurementToAngle() {
        positionMeasurement = PositionMeasurement.Angle;
    }

    public void setThetaUnit(BaseUnit thetaUnit) {
        if (thetaUnit.getDimension() != BaseUnit.Dimension.Angle) {
            return;
        }

        THETA_UNIT_PU = thetaUnit;

        updateUnits_pu();
    }

    public void setTimeUnit(BaseUnit timeUnit) {
        if (timeUnit.getDimension() != BaseUnit.Dimension.Time) {
            return;
        }

        TIME_UNIT_PU = timeUnit;

        updateUnits_pu();
    }

    public void setSecondTimeUnit(BaseUnit timeUnit) {
        if (timeUnit.getDimension() != BaseUnit.Dimension.Time) {
            return;
        }

        SECOND_TIME_UNIT_PU = timeUnit;

        updateUnits_pu();
    }

    public void setLengthUnit(BaseUnit lengthUnit) {
        if (lengthUnit.getDimension() != BaseUnit.Dimension.Length) {
            return;
        }

        LENGTH_UNIT_PU = lengthUnit;

        updateUnits_pu();
    }



    private void updateUnits_pu() {
        OMEGA_UNIT_PU = (new UnitBuilder()).num(THETA_UNIT_PU).denom(TIME_UNIT_PU).make();
        ALPHA_UNIT_PU = (new UnitBuilder()).num(THETA_UNIT_PU).denom(SECOND_TIME_UNIT_PU).make();

        RAD_PER_TIME = (new UnitBuilder()).num(Units.RAD).denom(TIME_UNIT_PU).make();
        RAD_PER_TIME2 = (new UnitBuilder()).num(Units.RAD).denom(SECOND_TIME_UNIT_PU).make();
        
        VEL_UNIT_PU = (new UnitBuilder()).num(LENGTH_UNIT_PU).denom(TIME_UNIT_PU).make();
        ACC_UNIT_PU = (new UnitBuilder()).num(LENGTH_UNIT_PU).denom(SECOND_TIME_UNIT_PU).make();
    }



    public Quantity toAngular(Quantity tangential) {
        return tangential.divide(radius).multiply(Units.RAD);
    }

    public Quantity toTangential(Quantity angular) {
        return angular.divide(Units.RAD).multiply(radius);
    }





    public abstract void setInverted(boolean invert);

    /**
     * Command the position of the motor to a specified amount of encoder ticks
     * 
     * @param ticks encoder ticks to command the motor to
     * @param controlMethod control method (MotionMagic or PID) to be used
     */
    protected abstract void cmdPosition_native(double val_nu, ControlType controlMethod);

    public void cmdPosition(double pos, ControlType controlMethod, int configID) {
        if (controlMethod.getVariable() != ControlType.Variable.Position) {
            return;
        }

        if (THETA_UNIT_NU == null) {
            return;
        }

        setMotionConfig(configID);

        if (positionMeasurement == PositionMeasurement.Angle) {
            Quantity quant = new Quantity(pos, THETA_UNIT_PU);

            cmdPosition_native(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        // existence of a theta unit guaranteed
        } else if (positionMeasurement == PositionMeasurement.Distance) {
            // l = r theta
            // theta = l / r and also radians are pretty rad
            Quantity quant = new Quantity(pos, LENGTH_UNIT_PU).divide(radius).multiply(Units.RAD);

            cmdPosition_native(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        }
    }

    public void cmdPosition(double pos, ControlType controlMethod, String configName) {
        Integer slotID = motionConfigNames.get(configName);

        if (slotID == null) {
            return; // rip ig
        } else {
            cmdPosition(pos, controlMethod, slotID);
        }
    }

    public void cmdPosition(double pos, ControlType controlMethod) {
        int configID = findMotionConfig(controlMethod);

        if (configID != -1) {
            cmdPosition(pos, controlMethod, configID);
        }
    }

    public void cmdPosition(Quantity quant, ControlType controlMethod, int configID) {
        if (controlMethod.getVariable() != ControlType.Variable.Position) {
            return;
        }

        // make sure there's an actual unit to use
        if (THETA_UNIT_NU == null) {
            return;
        }

        setMotionConfig(configID);

        if (quant.getUnit().isCompatible(THETA_UNIT_PU)) {
            cmdPosition_native(quant.to(THETA_UNIT_NU).getValue(), controlMethod);
        } else if (LENGTH_UNIT_PU != null && quant.getUnit().isCompatible(LENGTH_UNIT_PU)) {
            cmdPosition_native(toAngular(quant).to(THETA_UNIT_NU).getValue(), controlMethod);
        }
    }

    public void cmdPosition(Quantity quant, ControlType controlMethod, String configName) {
        Integer pidID = motionConfigNames.get(configName);

        if (pidID == null) {
            return;
        } else {
            cmdPosition(quant, controlMethod, pidID);
        }
    }

    public void cmdPosition(Quantity quant, ControlType controlMethod) {
        int configID = findMotionConfig(controlMethod);

        if (configID != -1) {
            cmdPosition(quant, controlMethod, configID);
        }
    }



    protected abstract void cmdVelocity_native(double vel);

    public void cmdVelocity(double vel, int configID) {
        if (OMEGA_UNIT_NU == null) {
            return;
        }

        setMotionConfig(configID);

        if (positionMeasurement == PositionMeasurement.Angle) {
            Quantity quant = new Quantity(vel, OMEGA_UNIT_PU);

            cmdVelocity_native(quant.to(OMEGA_UNIT_NU).getValue());
        // existence of a velocity unit guaranteed
        } else if (positionMeasurement == PositionMeasurement.Distance) {
            Quantity quant = new Quantity(vel, VEL_UNIT_PU);

            cmdVelocity_native(toAngular(quant).to(OMEGA_UNIT_NU).getValue());
        }
    }

    public void cmdVelocity(double vel, String configName) {
        Integer configID = motionConfigNames.get(configName);

        if (configID == null) {
            return; // rip ig
        } else {
            cmdVelocity(vel, configID);
        }
    }

    public void cmdVelocity(double vel) {
        int configID = findMotionConfig(ControlType.Velocity);

        if (configID != -1) {
            cmdVelocity(vel, configID);
        }
    }

    public void cmdVelocity(Quantity vel, int configID) {
        if (OMEGA_UNIT_NU == null) {
            return;
        }

        setMotionConfig(configID);

        if (vel.getUnit().isCompatible(OMEGA_UNIT_PU)) {
            cmdVelocity_native(vel.to(OMEGA_UNIT_NU).getValue());
        } else if (VEL_UNIT_PU != null && vel.getUnit().isCompatible(VEL_UNIT_PU)) {
            cmdVelocity_native(toAngular(vel).to(OMEGA_UNIT_NU).getValue());
        }
    }

    public void cmdVelocity(Quantity vel, String configName) {
        Integer configID = motionConfigNames.get(configName);

        if (configID == null) {
            return; // rip ig
        } else {
            cmdVelocity(vel, configID);
        }
    }

    public void cmdVelocity(Quantity vel) {
        int configID = findMotionConfig(ControlType.Velocity);

        if (configID != -1) {
            cmdVelocity(vel, configID);
        }
    }



    protected abstract void cmdPercent_native(double perc);

    public void cmdPercent(double perc) {
        cmdPercent_native(perc);
    }





    /** Type of encoder attached the to motor */
    protected SensorType sensor;

    public void addEncoder(SensorType sensor) {
        this.sensor = sensor;

        if (sensor instanceof QuadratureEncoder) {
            addQuadratureEncoder((QuadratureEncoder) sensor);
        }

        updateUnits_nu();
    }



    protected abstract void addQuadratureEncoder(QuadratureEncoder sensor);
    public abstract void setSensorPhase(boolean phase);



    /**
     * Return the position read on the encoder in ticks
     * 
     * @return position on the encoder in ticks
     */
    public abstract double getPosition_nu();



    /**
     * Return the position read on the encoder in preferred units
     * 
     * @return position on the encoder in preferred units
     */
    public double getPosition() {
    	if (THETA_UNIT_NU == null) {
    		return 0;
    	}
    	
        Quantity theta_nu = new Quantity(getPosition_nu(), THETA_UNIT_NU);

        if (positionMeasurement == PositionMeasurement.Angle) {
            return theta_nu.to(THETA_UNIT_PU).getValue();
        } else {
            // l = r theta but theta in rads that don't really exist but oh well
            return theta_nu.to(Units.RAD).divide(Units.RAD).multiply(radius).getValue();
        }
    }

    public double getPosition(BaseUnit unit) {
    	if (THETA_UNIT_NU == null) {
    		return 0;
    	}
    	
        Quantity theta_nu = new Quantity(getPosition_nu(), THETA_UNIT_NU);

        if (unit.getDimension() == Dimension.Length) {
            if (radius != null) {
                return theta_nu.to(THETA_UNIT_PU).divide(Units.RAD).multiply(radius).to(unit).getValue();
            } else {
                return 0; // rip
            }
        } else if (unit.getDimension() == Dimension.Angle) {
            return theta_nu.to(unit).getValue();
        } else {
            return 0; // rip
        }
    }

    



    protected abstract void setPosition_nu(double pos_nu);

    public void setPosition(double pos_pu) {
        if (positionMeasurement == PositionMeasurement.Angle) {
            setPosition_nu((new Quantity(pos_pu, THETA_UNIT_PU)).to(THETA_UNIT_NU).getValue());
        } else {
            setPosition_nu(toAngular(new Quantity(pos_pu, LENGTH_UNIT_PU)).to(THETA_UNIT_NU).getValue());
        }
    }

    public void setPosition(Quantity quant) {
        if (quant.getUnit().isCompatible(Dimension.Angle)) {
            setPosition_nu(quant.to(THETA_UNIT_NU).getValue());
        } else if (quant.getUnit().isCompatible(LENGTH_UNIT_PU)) {
            setPosition_nu(toAngular(quant).to(THETA_UNIT_NU).getValue());
        }
    }

    public void zero() {
        setPosition_nu(0);
    }



    /**
     * Get the velocity in native encoder units
     * 
     * @return velocity in native encoder units
     */
    public abstract double getVelocity_nu();

    /**
     * Get the velocity of the object in preferred units of the object
     */
    public double getVelocity() {
        // ticks per TIME_PERIOD
        Quantity vel_nu = new Quantity(getVelocity_nu(), OMEGA_UNIT_NU);

        if (positionMeasurement == PositionMeasurement.Angle) {
            return vel_nu.to(OMEGA_UNIT_PU).getValue();
        } else {
            // v = r omega but omega in rads that don't really exist but oh well
            return vel_nu.to(RAD_PER_TIME).divide(Units.RAD).multiply(radius).getValue();
        }
    }



    public abstract void setOpenLoopRampRate(double fullThrottleSec);
    public abstract void setClosedLoopRampRate(double fullThrottleSec);



    /**
     * Set the position in revolutions (OF ENCODER - not necessarily of thing being controller). 
     */
    //public abstract void cmdPosition(double revs);


    
    /** Get the voltage drop across the motor in volts (V) */
    public abstract double getVoltage();

    /** Get the percentage (0 to 1) of the robot's voltage that is seen by the motor controller*/
    public abstract double getPercentVoltage();

    public abstract double getCurrent();



    public abstract void follow(BBMotorController motorController);





    public void setSI() {
        setMeasurementToAngle();
        setThetaUnit(Units.RAD);
        setTimeUnit(Units.S);
        setSecondTimeUnit(Units.S);
    }










    protected boolean hasDiagnostics;
    protected DataWindow<MotorInfo> diagnosticData;
}