package frc.robot.utils.math.units;

import frc.robot.utils.math.units.BaseUnit.Dimension;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class Units {
    // Length units
    public static final BaseUnit IN = new BaseUnit(Dimension.Length, 1, "in");
    public static final BaseUnit FT = new BaseUnit(IN, 1.0/12, "ft");
    public static final BaseUnit CM = new BaseUnit(IN, 2.54, "cm");
    public static final BaseUnit M =  new BaseUnit(CM, 0.01, "m");
    public static final BaseUnit YARD    = new BaseUnit(FT, 1.0/3, "yd");
    public static final BaseUnit FURLONG = new BaseUnit(YARD, 1.0/220, "fur");
    public static final BaseUnit SMOOT = new BaseUnit(FT, 1.0/(5 + 7.0/12.0), "sm"); // 5 foot 7 per Smoot

    // Angles
    public static final BaseUnit RAD = new BaseUnit(Dimension.Angle, 1, "rad");
    public static final BaseUnit REV = new BaseUnit(RAD, 1.0/(2 * Math.PI), "rev");
    public static final BaseUnit DEG = new BaseUnit(REV, 360, "deg");

    // Mass
    public static final BaseUnit LB = new BaseUnit(Dimension.Mass, 1, "lb");
    public static final BaseUnit KG = new BaseUnit(LB, 0.453592, "kg");

    // Time
    public static final BaseUnit S   = new BaseUnit(Dimension.Time, 1, "s");
    public static final BaseUnit MIN = new BaseUnit(S, 1.0/60, "min");
    public static final BaseUnit MS  = new BaseUnit(S, 1000, "ms");
    public static final BaseUnit MS100 = new BaseUnit(MS, 1.0/100, "100ms");
    public static final BaseUnit HR   = new BaseUnit(MIN, 1.0/60, "hr");
    public static final BaseUnit DAY   = new BaseUnit(HR, 1.0/12, "day");
    public static final BaseUnit WEEK   = new BaseUnit(DAY, 1.0/7, "wk");
    public static final BaseUnit FORTNIGHT   = new BaseUnit(DAY, 1/2.0, "fn");

    // Velocity
    public static final Unit IN_PER_S = IN.divide(S);
    public static final Unit FT_PER_S = FT.divide(S);
    public static final Unit M_PER_S  = M.divide(S);
    public static final Unit FURLONGS_PER_FORTNIGHT = FURLONG.divide(FORTNIGHT);

    // Acceleration
    public static final Unit IN_PER_S2 = IN_PER_S.divide(S);
    public static final Unit FT_PER_S2 = FT_PER_S.divide(S);
    public static final Unit M_PER_S2  = M_PER_S.divide(S);
    public static final Unit G_ACC     = (new UnitBuilder()).num(M).denom(S, S).coeff(9.80665).name("g's").make();

    // Angular velocity
    public static final Unit RAD_PER_S = RAD.divide(S);
    public static final Unit DEG_PER_S = DEG.divide(S);
    public static final Unit REV_PER_S = REV.divide(S);
    public static final Unit RPM       = (new UnitBuilder()).num(REV).denom(MIN).name("RPM").make();

    // Angular acceleration
    public static final Unit RAD_PER_S2    = RAD_PER_S.divide(S);
    public static final Unit DEG_PER_S2    = DEG_PER_S.divide(S);
    public static final Unit REV_PER_S2    = REV_PER_S.divide(S);
    public static final Unit REV_PER_MIN_S = RPM.divide(S);
    public static final Unit REV_PER_MIN2  = RPM.divide(MIN);

    // energy
    public static final Unit J = (new UnitBuilder()).num(KG, M, M).denom(S, S).name("J").make();

    // torque
    public static final Unit Nm = new Unit(J, 1, "Nm"); // shhhhh

    // Momentum
    public static final Unit GB = (new UnitBuilder()).num(KG, M).denom(S).name("gb").make(); // 1 greenberg = 1 kgm/s
    
    // Force
    public static final Unit N = (new UnitBuilder()).num(Units.KG, Units.M).denom(Units.S, Units.S).name("N").make();

    // Current
    public static final BaseUnit A = new BaseUnit(Dimension.Current, 1, "A");

    // Charge
    public static final Unit C = (new UnitBuilder()).num(A, S).name("C").make();

    // Voltage
    public static final Unit V = (new UnitBuilder()).num(KG, M, M).denom(S, S, S, A).make();
    public static final Unit CTRE_VOLTAGE = new Unit(V, 12.2/1023.0, "CTRE Voltage"); // 1023 voltage units = 12.2V approx

    // Resistance
    // Java doesn't support Ω as a variable name
    public static final Unit Ohm = (new UnitBuilder()).num(V).denom(A).name("Ω").make();

    // Inductance
    public static final Unit H = (new UnitBuilder()).num(Ohm, S.getUnit()).name("H").make();
}