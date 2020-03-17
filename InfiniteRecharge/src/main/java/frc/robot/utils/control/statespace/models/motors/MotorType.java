package frc.robot.utils.control.statespace.models.motors;

import org.ejml.data.DMatrixRMaj;

import frc.robot.utils.math.units.Quantity;
import frc.robot.utils.math.units.Units;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class MotorType {
    public static final MotorType BAG = new BAG();
    public static final MotorType CIM = new CIM();
    public static final MotorType Falcon500 = new Falcon500();
    public static final MotorType MiniCIM = new MiniCIM();
    public static final MotorType NEO = new NEO();
    public static final MotorType pro775 = new pro775();





    // model of a motor's physics ignoring inductance (small enough to be irrelevant in FRC, typically)
    private final Quantity b;
    private final Quantity R;
    private final Quantity K_T;
    private final Quantity K_W;



    public MotorType(double b, double R, double Kt, double Kw) {
        this.b = new Quantity(b, Units.Nm.divide(Units.RAD_PER_S));
        this.R = new Quantity(R, Units.Ohm);
        K_T = new Quantity(Kt, Units.Nm.divide(Units.A));
        K_W = new Quantity(Kw, Units.V.divide(Units.RAD_PER_S));
    }

    

    public Quantity getB() { return b; }
    public Quantity getR() { return R; }
    public Quantity getKT() { return K_T; }
    public Quantity getKW() { return K_W; }
    
    public Quantity getKF() {
        return (b.multiply(R).divide(K_T)).add(K_W);
    }
}