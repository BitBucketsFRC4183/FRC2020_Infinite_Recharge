package frc.robot.utils.math.units;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class Quantity {
    private final double VALUE;
    private final Unit UNIT;



    public Quantity(double value, Unit unit) {
        VALUE = value;
        UNIT = unit;
    }

    public Quantity(double value, BaseUnit unit) {
        this(value, unit.getUnit());
    }



    public Unit getUnit() { return UNIT; }
    public double getValue() { return VALUE; }

    public Quantity to(Unit unit2) {
        return new Quantity(VALUE / UNIT.per(unit2), unit2);
    }

    public Quantity to(BaseUnit unit2) {
        return to(unit2.getUnit());
    }

    public Quantity multiply(Quantity q2) {
        return new Quantity(VALUE * q2.getValue(), UNIT.multiply(q2.getUnit()));
    }

    public Quantity multiply(Unit unit) {
        return new Quantity(VALUE, UNIT.multiply(unit));
    }

    public Quantity multiply(BaseUnit unit) {
        return multiply(unit.getUnit());
    }

    public Quantity divide(Quantity q2) {
        return new Quantity(VALUE / q2.getValue(), UNIT.divide(q2.getUnit()));
    }

    public Quantity divide(Unit unit) {
        return new Quantity(VALUE, UNIT.divide(unit));
    }

    public Quantity divide(BaseUnit unit) {
        return divide(unit.getUnit());
    }



    public Quantity add(Quantity q2) {
        if (UNIT.isCompatible(q2.getUnit())) {
            return new Quantity(VALUE + q2.to(UNIT).getValue(), UNIT);
        } else {
            return null;
        }
    }

    public Quantity subtract(Quantity q2) {
        if (UNIT.isCompatible(q2.getUnit())) {
            return new Quantity(VALUE - q2.to(UNIT).getValue(), UNIT);
        } else {
            return null;
        }
    }
    
    @Override
    public String toString() {
    	return VALUE + " " + UNIT.toString();
    }
}