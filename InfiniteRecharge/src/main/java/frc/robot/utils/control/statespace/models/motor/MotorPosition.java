package frc.robot.utils.control.statespace.models.motor;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.control.statespace.models.lti.CLTIModel;
import frc.robot.utils.control.statespace.models.motors.MotorType;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class MotorPosition extends CLTIModel {
    public MotorPosition(double t, MotorType type, double I) {
        super(
            getA(type, I),
            getB(type, I),
            t
        );
    }



    public static SimpleMatrix getA(MotorType type, double I) {
        double kw = type.getKW().getValue();
        double kt = type.getKT().getValue();
        double b = type.getB().getValue();
        double R = type.getR().getValue();

        return new SimpleMatrix(new double[][] {
            new double[] {0, 1},
            new double[] {0, -((kt * kw) / R + b)/I}
        });
    }

    public static SimpleMatrix getB(MotorType type, double I) {
        double kt = type.getKT().getValue();
        double R = type.getR().getValue();

        return new SimpleMatrix(new double[][] {
            new double[] {0},
            new double[] {kt/(I*R)}
        });
    }
}