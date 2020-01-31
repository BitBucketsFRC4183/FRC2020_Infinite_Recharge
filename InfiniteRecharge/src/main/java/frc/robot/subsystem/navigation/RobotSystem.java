package frc.robot.subsystem.navigation;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ltif.CLTIFModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;

public class RobotSystem {
    public final double R_R = 0.3145536;
    public final double M = 39.3051396;
    public final double I_R = 2.1621037;
    public final double R_W = 0.0762;
    public final double I_W = 0.001;

    public final MotorType type = MotorType.Falcon500;
    public final double Kt = type.getKT().getValue();
    public final double Kw = type.getKW().getValue();
    public final double R = type.getR().getValue();
    public final double b = type.getB().getValue();

    public final double T_F_L = -0.1;
    public final double T_F_R = -0.1;

    public final double F_F_L = T_F_L / R_W;
    public final double F_F_R = T_F_R / R_W;

    public final double G = 10 + 8.0/9;
    public final double e = 0.81;



    private CLTIFModel model;

    public RobotSystem() {
    }

    public CLTIFModel getModel() {
        double t0 = System.nanoTime();
        double a = (1/M + R_R * R_R / I_R);
        double b = (1/M - R_R * R_R / I_R);

        double Cv = -(Kw * Kt * G * G * e / (R_R * R_W * R_W) + b * G * G * e / (R_W * R_W));
        double CV = G * e * Kt / (R_R * R_W);
        double t1 = System.nanoTime();
        //System.out.println("RobotSystem Setup time: " + (t1 - t0) / 1000000 + "ms");

        SimpleMatrix Ac = new SimpleMatrix(2, 2, true, new double[] {a*Cv, b*Cv, b*Cv, a*Cv});

        SimpleMatrix Bc = new SimpleMatrix(2, 2, true, new double[] {a*CV, b*CV, b*CV, a*CV});

        SimpleMatrix Fc = new SimpleMatrix(2, 1, true, new double[] {a*F_F_L + b*F_F_R, b*F_F_L + a*F_F_R});

        double t2 = System.nanoTime();
        //System.out.println("RobotSystem continuous matrix time: " + (t2 - t1) / 1000000 + "ms");

        model = new CLTIFModel(Ac, Bc, Fc, 0.02);
        double t3 = System.nanoTime();
        //System.out.println("RobotSystem C2D time: " + (t3 - t2) / 1000000 + "ms");

        return model;
    }
}