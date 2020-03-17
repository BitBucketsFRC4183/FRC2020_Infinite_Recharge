package frc.robot.utils.control.statespace.models;

import org.ejml.simple.SimpleMatrix;

import frc.robot.utils.math.MathUtils;



/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
// MATLAB's implementation  of continuous to discrete system equation
public class C2D {
    public static ABFTriple c2d(ABFTriple ABF, double dt) {
        return c2d(
            ABF.getA(),
            ABF.getB(),
            ABF.getF(),
            dt
        );
    }



    public static ABFTriple c2d(SimpleMatrix A, SimpleMatrix B, SimpleMatrix F, double dt) {
        int n = A.numRows();
        SimpleMatrix s = getS(A, dt);

        SimpleMatrix gamma = getGamma(s, n);

        return new ABFTriple(
            getA(s, n),
            gamma.mult(B),
            gamma.mult(F)
        );
    }

    

    public static ABCouple c2d(ABCouple AB, double dt) {
        return c2d(
            AB.getA(),
            AB.getB(),
            dt
        );
    }

    public static ABCouple c2d(SimpleMatrix A, SimpleMatrix B, double dt) {
        int n = A.numRows();
        SimpleMatrix s = getS(A, dt);

        SimpleMatrix gamma = getGamma(s, n);

        return new ABCouple(
            getA(s, n),
            gamma.mult(B)
        );
    }


    
    public static SimpleMatrix getS(SimpleMatrix A, double dt) {
        int n = A.numRows();

        /*
            s = expm([A I; 0 0]*t)
            Ad = s[0:n, 0:n]
            Bd = s[0:n, n:2n]*B
            Fd = s[0:n, n:2n]*F
        */
        SimpleMatrix s = new SimpleMatrix(2*n, 2*n);
        s.insertIntoThis(0, 0, A);
        s.insertIntoThis(0, n, SimpleMatrix.identity(n));
        return MathUtils.expm(s.scale(dt));
    }

    public static SimpleMatrix getA(SimpleMatrix s, int n) {
        return s.extractMatrix(0, n, 0, n);
    }

    public static SimpleMatrix getGamma(SimpleMatrix s, int n) {
        return s.extractMatrix(0, n, n, 2*n);
    }
}