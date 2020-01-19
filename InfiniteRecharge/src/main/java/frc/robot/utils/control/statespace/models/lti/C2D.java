package frc.robot.utils.control.statespace.models.lti;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.math.MathUtils;

// MATLAB's implementation  of continuous to discrete system equation
public class C2D {
    public static SimpleMatrix getS(SimpleMatrix A, SimpleMatrix B, double dt) {
        int m = A.numRows();
        int nb = B.numCols();

        SimpleMatrix s = new SimpleMatrix(m + nb, m + nb);
        s.insertIntoThis(0, 0, A);
        s.insertIntoThis(0, m, B);
        s.insertIntoThis(m, 0, new SimpleMatrix(nb, m + nb));

        return MathUtils.expm(s);
    }



    public static SimpleMatrix getA(SimpleMatrix s, int m) {
        return s.extractMatrix(0, m, 0, m);
    }

    public static SimpleMatrix getB(SimpleMatrix s, int m, int nb) {
        return s.extractMatrix(0, m, m, m + nb);
    }



    public static SimpleMatrix[] c2d(SimpleMatrix A, SimpleMatrix B, double dt) {
        SimpleMatrix s = getS(A, B, dt);

        SimpleMatrix Ad = getA(s, A.numRows());
        SimpleMatrix Bd = getB(s, B.numRows(), B.numCols());

        return new SimpleMatrix[] {Ad, Bd};
    }
}