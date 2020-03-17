package frc.robot.utils.control.statespace.models;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class ABFTriple {
    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix F;



    public ABFTriple(SimpleMatrix A, SimpleMatrix B, SimpleMatrix F) {
        this.A = A;
        this.B = B;
        this.F = F;
    }



    public SimpleMatrix getA() { return A; }
    public SimpleMatrix getB() { return B; }
    public SimpleMatrix getF() { return F; }



    public ABCouple getCouple() {
        return new ABCouple(A, B);
    }



    public String toString() {
        return "A:\n" + A.toString() + "\n\nB:\n" + B.toString() + "\n\nF:\n" + F.toString();
    }
}