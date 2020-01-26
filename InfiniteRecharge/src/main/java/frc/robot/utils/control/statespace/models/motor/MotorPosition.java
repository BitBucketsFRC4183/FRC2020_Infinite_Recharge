package frc.robot.utils.control.statespace.models.motor;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.lti.CLTIModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;

public class MotorPosition extends CLTIModel {
    public MotorPosition(double t, MotorType type, double I) {
        super(
            getA(type, I),
            getB(type, I),
            t
        );
    }



    private static SimpleMatrix getA(MotorType type, double I) {
        double kw = type.getKW().getValue();
        double kt = type.getKT().getValue();
        double b = type.getB().getValue();
        double R = type.getR().getValue();

        return new SimpleMatrix(new double[][] {
            new double[] {0, 1},
            new double[] {0, -((kt * kw) / R + b)/I}
        });
    }

    private static SimpleMatrix getB(MotorType type, double I) {
        double kt = type.getKT().getValue();
        double R = type.getR().getValue();

        return new SimpleMatrix(new double[][] {
            new double[] {0},
            new double[] {0, kt/(I*R)}
        });
    }
}