package frc.robot.utils.math.units;

import java.util.ArrayList;
import java.util.HashMap;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class BaseUnit {
    /** Dimensions for BaseUnits to measure */
    public enum Dimension {
        Length,
        Angle,
        Mass,
        Current,
        Time
    }





    /** how many of this unit in 1 standard unit */
    private double perStandard;
    /** Dimension this unit measures */
    private final Dimension DIM;



    private final Unit UNIT;



    /** Whether to store conversions to other units. Trade-off between computation time and memory space. */
    protected boolean storeConversions = false;
    /** Stored conversions from a Unit of the same Dimension(s) to another */
    protected HashMap<BaseUnit, Double> conversions = new HashMap<BaseUnit, Double>();
    
    
    
    private final String NAME;



    /**
     * Create a BaseUnit
     * 
     * @param dim dimension the unit measures
     */
    public BaseUnit(Dimension dim, String name) {
        this.DIM = dim;

        ArrayList<BaseUnit> nums = new ArrayList<BaseUnit>();
        nums.add(this);
        ArrayList<BaseUnit> denoms = new ArrayList<BaseUnit>();

        UNIT = new Unit(nums, denoms);
        
        NAME = name;
    }

    /**
     * Create a BaseUnit
     * 
     * @param dim dimension the unit measures
     * @param perStandard how many of this unit are there per standard unit
     */
    public BaseUnit(Dimension dim, double perStandard, String name) {
        this(dim, name);

        this.perStandard = perStandard;
    }

    /**
     * Create a BaseUnit based off of conversion to another
     * 
     * @param unit other unit that conversion is specified to
     * @param per how many of this BaseUnit to other unit
     */
    public BaseUnit(BaseUnit unit, double per, String name) {
        this(unit.getDimension(), name);

        setPer(unit, per);
    }
    
    
    
    public String getName() {
    	return NAME;
    }
    
    @Override
    public String toString() { return getName(); }





    public Unit getUnit() {
        return UNIT;
    }


    
    /**
     * Return the dimension this unit measures
     * 
     * @return dimension unit measures
     */
    public Dimension getDimension() {
        return DIM;
    }



    public boolean isCompatible(Unit unit2) {
        return unit2.isCompatible(this);
    }

    public boolean isCompatible(BaseUnit unit2) {
        return DIM == unit2.getDimension();
    }

    public boolean isCompatible(Dimension dim) {
        return DIM == dim;
    }



    /**
     * Sets how many of this unit are in a standard unit
     * 
     * @param perStandard how many of this unit are in the corresponding standard
     */
    public void setPerStandard(double perStandard) {
        this.perStandard = perStandard;
    }
    
    public double getPerStandard() {
    	return perStandard;
    }

    /**
     * Set conversions based on conversion to another unit
     * 
     * @param bu other unit to base conversions off of
     * @param per amount of this unit per 1 bu
     */
    public void setPer(BaseUnit unit2, double per) {
        // make sure dimensions are compatible
        if (!isCompatible(unit2)) {
            return;
        }

        // if storing conversions, might as well store this given one
        if (storeConversions) {
            conversions.put(unit2, per);
        }

        // units / 1 bu * bu/std = units/std
        setPerStandard(per * unit2.perStandard());
    }

    public void setPer(Unit unit2, double per) {
        if (!isCompatible(unit2)) {
            return;
        }

        // if compatible, then only one thing in numerator should be
        // a compatible BaseUnit
        setPer(unit2.getNumeratorList().get(0), per / unit2.getCoefficient());
    }

    /**
     * Get the amount of this unit in the corresponding standard
     */
    public double perStandard() {
        return perStandard;
    }



    /**
     * Determine how many of this unit are in another of the same dimension
     * 
     * @param bu other BaseUnit to determine conversion to
     * @return how many of this unit are in bu, or 0 if dimensions are incompatible
     */
    public double per(BaseUnit unit2) {
        // don't double calculate if you already know the conversion
        if (storeConversions) {
            Double conv = conversions.get(unit2);

            // if you know the conversion, return it
            if (conv != null) {
                return conv;
            }
        }

        // return 0 if dimensions are incompatible
        // not needed at start because only stores comptatible BaseUnit conversions
        if (unit2.getDimension() != DIM) {
            return 0; // ig
        }

        // x unit = 1 unit2
        // x = unit2 / unit
        // x = unit2 / standard * standard/unit
        // unit perStandard = 1 standard
        // unit/standard = 1/perStandard
        // standard/unit = perStandard
        // standard = 
        double per = perStandard / unit2.perStandard();
        
        // if storing conversions, store this one now that the unit knows it
        if (storeConversions) {
            conversions.put(unit2, per);
        }

        return per;
    }

    public double per(Unit unit2) {
        if (!isCompatible(unit2)) {
            return 0;
        }

        BaseUnit bu = unit2.getNumeratorList().get(0);

        // don't double calculate if you already know the conversion
        if (storeConversions) {
            Double conv = conversions.get(bu);

            // if you know the conversion, return it
            if (conv != null) {
                // x in/(2*ft) = 1
                // x = 1 * (2ft)/in = 2 ft/in = 2 * 12 = 24
                return unit2.getCoefficient() * conv;
            }
        }

        // units/std * std/units2 = units/units2
        double per = perStandard / bu.perStandard() * unit2.getCoefficient();
        // if storing conversions, store this one now that the unit knows it
        if (storeConversions) {
            conversions.put(bu, per);
        }

        return per;
    }

    public Unit multiply(BaseUnit unit2) {
        return getUnit().multiply(unit2.getUnit());
    }

    public Unit multiply(Unit unit2) {
// haha funny FRC number 254
        return getUnit().multiply(unit2);
    }

    public Unit divide(BaseUnit unit2) {
        return getUnit().divide(unit2.getUnit());
    }

    public Unit divide(Unit unit2) {
        return getUnit().divide(unit2);
    }
}