package frc.robot.utils.control.statespace.models.motor;

import org.ejml.data.DMatrixRMaj;

import frc.robot.utils.control.statespace.models.lti.LTIModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;



public class MotorPosition extends LTIModel {
    public MotorPosition(double t, MotorType type, double I, double ts, double tk) {
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

        DMatrixRMaj Ac = new DMatrixRMaj(new double[][] {
            new double[] {0, 1},
            new double[] {0, -(kt * kw / R + b)/I}
        });

        DMatrixRMaj Bc = new DMatrixRMaj(new double[][] {
            new double[] {0},
            new double[] {0, kt/(I*R)}
        });

        DMatrixRMaj C = new DMatrixRMaj(new double[][] {
            new double[] {1, 1}, // ehhh, can't ~technically~ measure velocity
        });
    }
}