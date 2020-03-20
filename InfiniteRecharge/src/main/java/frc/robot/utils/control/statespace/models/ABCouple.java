package frc.robot.utils.control.statespace.models;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class ABCouple {
    private SimpleMatrix A;
    private SimpleMatrix B;



    public ABCouple(SimpleMatrix A, SimpleMatrix B) {
        this.A = A;
        this.B = B;
    }



    public SimpleMatrix getA() { return A; }
    public SimpleMatrix getB() { return B; }
}