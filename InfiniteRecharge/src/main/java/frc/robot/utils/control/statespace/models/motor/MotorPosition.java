package frc.robot.utils.control.statespace.models.motor;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.models.lti.C2D;
import frc.robot.utils.control.statespace.models.lti.LTIModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;

public class MotorPosition extends LTIModel {
    public MotorPosition(double t, MotorType type, double I, double ts, double tk) throws StateSpaceException {
        super(getSys(t, type, I, ts, tk));
    }



    private static SimpleMatrix[] getSys(double t, MotorType type, double I, double ts, double tk) {
        double kw = type.getKW().getValue();
        double kt = type.getKT().getValue();
        double b = type.getB().getValue();
        double R = type.getR().getValue();

        // Assuming no inductance, we can say that
        // V = Kw w + R i
        // ->
        // i = (V - Kw w) / R
        //
        // T = Ia = Kt i - b w - tf (frictional torque)
        // Ia = Kt (V - Kw w) / R - b w - tf
        // Ia = (Kt/R) V - (Kt Kw / R + b) w - tf
        // a = (Kt/RI) V - (Kt Kw / RI + b/I) w - tf/I
        // x = [theta, velocity]

        SimpleMatrix Ac = new SimpleMatrix(new double[][] {
            new double[] {0, 1},
            new double[] {0, -(kt * kw / R + b)/I}
        });

        SimpleMatrix Bc = new SimpleMatrix(new double[][] {
            new double[] {0},
            new double[] {0, kt/(I*R)}
        });

        SimpleMatrix C = new SimpleMatrix(new double[][] {
            new double[] {1, 1}, // ehhh, can't ~technically~ measure velocity
        });

        SimpleMatrix[] sys = C2D.c2d(Ac, Bc, t);
        return new SimpleMatrix[] {sys[0], sys[1], C};
    }
}