package frc.robot.utils.control.statespace.observer;

import org.ejml.simple.SimpleMatrix;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class SimpleOutputObserver extends LinearizedOutputObserver {
    private final SimpleMatrix C;
    private final SimpleMatrix D;

    public SimpleOutputObserver(int numOutputs, SimpleMatrix C, SimpleMatrix D) {
        super(numOutputs);

        this.C = C;
        this.D = D;
    }



    @Override
    public SimpleMatrix getOutput(SimpleMatrix state, SimpleMatrix input, double t, int k) {
        return (C.mult(state)).plus(D.mult(input));
    }

    @Override
    public SimpleMatrix getJacobian(SimpleMatrix state, SimpleMatrix input, double t, int k) {
        return C;
    }
}