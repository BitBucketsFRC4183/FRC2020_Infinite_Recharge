package frc.robot.subsystem.navigation;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.ltif.CLTIFModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class RobotSystem {
    public final double R_R = 0.3145536;
    public final double M = 39.3051396;
    public final double I_R = 2.1621037;
    public final double R_W = 0.0762;
    //public final double I_W = 0.001;

    public final MotorType type = MotorType.Falcon500;
    public final double Kt = type.getKT().getValue();
    public final double Kw = type.getKW().getValue();
    public final double R = type.getR().getValue();
    public final double b = 0;//type.getB().getValue();

    public final double T_F_L = -0.1;
    public final double T_F_R = -0.1;

    public final double F_F_L = T_F_L / R_W;
    public final double F_F_R = T_F_R / R_W;

    public final double G = 10 + 8.0/9;
    public final double e = 1;



    private CLTIFModel model;

    public RobotSystem() {
    }

    public CLTIFModel getModel(double v0, double theta0) {
        double aM = (1/M + R_R * R_R / I_R);
        double bM = (1/M - R_R * R_R / I_R);
        double cM = 1.0/(2*R_R);

        double Cv = -(Kw * Kt * G * G * e / (R * R_W * R_W) + b * G * G * e / (R_W * R_W));
        double CV = G * e * Kt / (R * R_W);

        // x' = (vl + vr)/2 cos theta
        // x' = 1/2 cos theta vl + 1/2 cos theta vr / 2 - (vl + vr)/2 sin theta * theta + (vl+vr)/2 sin theta * theta0
        SimpleMatrix Ac = new SimpleMatrix(new double[][] {
            new double[] {0, 0, -v0 * Math.sin(theta0), Math.cos(theta0) / 2, Math.cos(theta0) / 2},
            new double[] {0, 0, v0 * Math.cos(theta0), Math.sin(theta0) / 2, Math.sin(theta0) / 2},
            new double[] {0, 0, 0, -cM,    cM},
            new double[] {0, 0, 0, aM*Cv, bM*Cv},
            new double[] {0, 0, 0, bM*Cv, aM*Cv}
        });

        SimpleMatrix Bc = new SimpleMatrix(new double[][] {
            new double[] {0, 0},
            new double[] {0, 0},
            new double[] {0, 0},
            new double[] {aM*CV, bM*CV},
            new double[] {bM*CV, aM*CV}
        });

        SimpleMatrix Fc = new SimpleMatrix(new double[][] {
            new double[] {v0 * theta0 * Math.sin(theta0)},
            new double[] {-v0 * theta0 * Math.cos(theta0)},
            new double[] {0},
            new double[] {aM*F_F_L + bM*F_F_R},
            new double[] {bM*F_F_L + aM*F_F_R}
        });

        //System.out.println("RobotSystem continuous matrix time: " + (t2 - t1) / 1000000 + "ms");

        model = new CLTIFModel(Ac, Bc, Fc, 0.02);

        return model;
    }
}