package frc.robot.utils.control.statespace.models.motors;

import org.ejml.data.DMatrixRMaj;

import frc.robot.utils.math.units.Quantity;
import frc.robot.utils.math.units.Units;

public class Motor {
    public static final Motor BAG = new BAG();
    public static final Motor CIM = new CIM();
    public static final Motor Falcon500 = new Falcon500();
    public static final Motor MiniCIM = new MiniCIM();
    public static final Motor NEO = new NEO();
    public static final Motor pro775 = new pro775();





    private final DMatrixRMaj A;
    private final DMatrixRMaj B;
    private final DMatrixRMaj C;

    private final Quantity b;
    private final Quantity R;
    private final Quantity K_T;
    private final Quantity K_W;



    public Motor(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, double b, double R, double Kt, double Kw) {
        this.A = A;
        this.B = B;
        this.C = C;

        this.b = new Quantity(b, Units.Nm.divide(Units.RAD_PER_S));
        this.R = new Quantity(b, Units.Ohm);
        K_T = new Quantity(Kt, Units.Nm.divide(Units.A));
        K_W = new Quantity(Kw, Units.V.divide(Units.RAD_PER_S));
    }

    

    public Quantity getKF() {
        return (b.multiply(R).divide(K_T)).add(K_W);
    }





    public DMatrixRMaj getA() { return A; }
    public DMatrixRMaj getB() { return B; }
    public DMatrixRMaj getC() { return C; }
}